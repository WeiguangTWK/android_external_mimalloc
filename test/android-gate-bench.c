/*
 * Copyright (C) 2026 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mimalloc_adapt.h"

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#define MAX_BENCH_THREADS 16
#define BENCHMARK_SAMPLES 3
#define BENCHMARK_DURATION_NS 2000000000ULL
#define WARMUP_DURATION_NS 250000000ULL
#define CLOCK_CHECK_INTERVAL 1024U

typedef enum benchmark_mode_e {
  BENCH_DIRECT,
  BENCH_GATE,
  BENCH_OLD_MUTEX,
} benchmark_mode_t;

typedef struct benchmark_arg_s {
  benchmark_mode_t mode;
  uintptr_t seed;
  pthread_barrier_t* barrier;
  const uint64_t* deadline_ns;
  uint64_t completed_pairs;
} benchmark_arg_t;

typedef struct benchmark_result_s {
  double pairs_per_second;
  double nanoseconds_per_pair;
} benchmark_result_t;

static pthread_mutex_t old_mutex = PTHREAD_MUTEX_INITIALIZER;
static size_t old_active_calls;

static uint64_t monotonic_nanoseconds(void);

static void fail(const char* operation, int err) {
  fprintf(stderr, "%s failed: %d\n", operation, err);
  abort();
}

static void old_operation_begin(void) {
  int err = pthread_mutex_lock(&old_mutex);
  if (err != 0) fail("pthread_mutex_lock", err);
  old_active_calls++;
  err = pthread_mutex_unlock(&old_mutex);
  if (err != 0) fail("pthread_mutex_unlock", err);
}

static void old_operation_end(void) {
  int err = pthread_mutex_lock(&old_mutex);
  if (err != 0) fail("pthread_mutex_lock", err);
  old_active_calls--;
  err = pthread_mutex_unlock(&old_mutex);
  if (err != 0) fail("pthread_mutex_unlock", err);
}

static void* benchmark_worker(void* argument) {
  benchmark_arg_t* arg = (benchmark_arg_t*)argument;
  uintptr_t seed = arg->seed;
  uint64_t completed = 0;
  int err = pthread_barrier_wait(arg->barrier);
  if (err != 0 && err != PTHREAD_BARRIER_SERIAL_THREAD) fail("pthread_barrier_wait", err);

  do {
    for (size_t i = 0; i < CLOCK_CHECK_INTERVAL; i++) {
      size_t size = ((seed++ * 17) & 1023) + 1;
      void* p;
      if (arg->mode == BENCH_DIRECT) {
        p = mi_malloc(size);
        mi_free(p);
      } else if (arg->mode == BENCH_GATE) {
        p = mimalloc_malloc(size);
        mimalloc_free(p);
      } else {
        old_operation_begin();
        p = mi_malloc(size);
        old_operation_end();
        old_operation_begin();
        mi_free(p);
        old_operation_end();
      }
      if (p == NULL) abort();
    }
    completed += CLOCK_CHECK_INTERVAL;
  } while (monotonic_nanoseconds() < *arg->deadline_ns);

  arg->completed_pairs = completed;
  return NULL;
}

static uint64_t monotonic_nanoseconds(void) {
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) fail("clock_gettime", errno);
  return (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
}

static benchmark_result_t run_benchmark(benchmark_mode_t mode, size_t thread_count,
                                        uint64_t duration_ns) {
  pthread_t threads[MAX_BENCH_THREADS];
  benchmark_arg_t args[MAX_BENCH_THREADS];
  pthread_barrier_t barrier;
  uint64_t deadline_ns = 0;

  int err = pthread_barrier_init(&barrier, NULL, (unsigned)thread_count + 1);
  if (err != 0) fail("pthread_barrier_init", err);

  for (size_t i = 0; i < thread_count; i++) {
    args[i].mode = mode;
    args[i].seed = i + 1;
    args[i].barrier = &barrier;
    args[i].deadline_ns = &deadline_ns;
    args[i].completed_pairs = 0;
    err = pthread_create(&threads[i], NULL, benchmark_worker, &args[i]);
    if (err != 0) fail("pthread_create", err);
  }

  uint64_t start = monotonic_nanoseconds();
  deadline_ns = start + duration_ns;
  err = pthread_barrier_wait(&barrier);
  if (err != 0 && err != PTHREAD_BARRIER_SERIAL_THREAD) fail("pthread_barrier_wait", err);
  for (size_t i = 0; i < thread_count; i++) {
    err = pthread_join(threads[i], NULL);
    if (err != 0) fail("pthread_join", err);
  }
  uint64_t elapsed = monotonic_nanoseconds() - start;
  pthread_barrier_destroy(&barrier);

  uint64_t pairs = 0;
  for (size_t i = 0; i < thread_count; i++) {
    pairs += args[i].completed_pairs;
  }

  benchmark_result_t result;
  result.pairs_per_second = (double)pairs * 1000000000.0 / (double)elapsed;
  result.nanoseconds_per_pair = (double)elapsed / (double)pairs;
  return result;
}

static void sort_results(benchmark_result_t results[BENCHMARK_SAMPLES]) {
  for (size_t i = 1; i < BENCHMARK_SAMPLES; i++) {
    benchmark_result_t value = results[i];
    size_t j = i;
    while (j > 0 && results[j - 1].nanoseconds_per_pair > value.nanoseconds_per_pair) {
      results[j] = results[j - 1];
      j--;
    }
    results[j] = value;
  }
}

static void print_result(const char* name,
                         benchmark_result_t results[BENCHMARK_SAMPLES]) {
  sort_results(results);
  const benchmark_result_t* median = &results[BENCHMARK_SAMPLES / 2];
  printf("  %-10s %12.0f pairs/s  %8.2f ns/pair  range %8.2f..%8.2f\n", name,
         median->pairs_per_second, median->nanoseconds_per_pair,
         results[0].nanoseconds_per_pair,
         results[BENCHMARK_SAMPLES - 1].nanoseconds_per_pair);
}

int main(void) {
  static const char* const mode_names[] = {"direct", "gate", "old-mutex"};
  static const size_t thread_counts[] = {1, 2, 4, 8, 16};
  long online_cpus = sysconf(_SC_NPROCESSORS_ONLN);
  printf("online_cpus=%ld, samples=%u, sample_duration=%.2fs, warmup=%.2fs\n",
         online_cpus, BENCHMARK_SAMPLES,
         (double)BENCHMARK_DURATION_NS / 1000000000.0,
         (double)WARMUP_DURATION_NS / 1000000000.0);

  for (size_t i = 0; i < sizeof(thread_counts) / sizeof(thread_counts[0]); i++) {
    size_t count = thread_counts[i];
    benchmark_result_t results[3][BENCHMARK_SAMPLES];

    // Initialize allocator state and raise the device out of its idle clock
    // state before collecting samples.
    for (size_t mode = 0; mode < 3; mode++) {
      (void)run_benchmark((benchmark_mode_t)mode, count, WARMUP_DURATION_NS);
    }

    // Rotate the mode order on every sample to distribute thermal and DVFS
    // effects instead of always favoring the first mode.
    for (size_t sample = 0; sample < BENCHMARK_SAMPLES; sample++) {
      for (size_t offset = 0; offset < 3; offset++) {
        size_t mode = (sample + offset) % 3;
        results[mode][sample] =
            run_benchmark((benchmark_mode_t)mode, count, BENCHMARK_DURATION_NS);
      }
    }

    for (size_t mode = 0; mode < 3; mode++) {
      sort_results(results[mode]);
    }

    const double direct_ns = results[BENCH_DIRECT][BENCHMARK_SAMPLES / 2].nanoseconds_per_pair;
    const double gate_ns = results[BENCH_GATE][BENCHMARK_SAMPLES / 2].nanoseconds_per_pair;
    const double old_ns = results[BENCH_OLD_MUTEX][BENCHMARK_SAMPLES / 2].nanoseconds_per_pair;

    printf("threads=%zu\n", count);
    for (size_t mode = 0; mode < 3; mode++) {
      print_result(mode_names[mode], results[mode]);
    }
    printf("  gate overhead vs direct: %+6.2f%%; gate speedup vs old-mutex: %.2fx\n",
           (gate_ns / direct_ns - 1.0) * 100.0, old_ns / gate_ns);
  }
  return 0;
}
