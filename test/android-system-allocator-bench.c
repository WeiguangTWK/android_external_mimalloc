/*
 * Copyright (C) 2026 Neko LineageOS
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

// This benchmark deliberately uses only libc's public allocator API. Keep it
// independent from mimalloc headers and libraries so one identical binary can
// compare Scudo and mimalloc system images.

#include <errno.h>
#include <malloc.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

#define MAX_BENCH_THREADS 16U
#define TRACE_LENGTH 8192U
#define LIVE_SLOTS_TOTAL 4096U
#define REMOTE_BATCH_SIZE 1024U
#define MAX_REMOTE_PAIRS (MAX_BENCH_THREADS / 2U)
#define MAX_LATENCY_SAMPLES 32768U
#define LATENCY_SAMPLE_INTERVAL 1009U
#define CLOCK_CHECK_INTERVAL 256U

#define BENCHMARK_SAMPLES 5U
#define BENCHMARK_DURATION_NS 1000000000ULL
#define WARMUP_DURATION_NS 250000000ULL

typedef enum workload_e {
  WORKLOAD_LOCAL,
  WORKLOAD_CHURN,
  WORKLOAD_REMOTE,
  WORKLOAD_COUNT,
} workload_t;

typedef enum size_profile_e {
  PROFILE_SMALL,
  PROFILE_MIXED,
  PROFILE_PAGE,
  PROFILE_COUNT,
} size_profile_t;

typedef struct trace_entry_s {
  uint32_t size;
  uint32_t slot;
} trace_entry_t;

typedef struct memory_stats_s {
  size_t rss_kb;
  size_t pss_kb;
  size_t private_dirty_kb;
} memory_stats_t;

typedef struct remote_pair_s {
  pthread_mutex_t mutex;
  pthread_cond_t condition;
  bool ready;
  bool done;
  void* pointers[REMOTE_BATCH_SIZE];
} remote_pair_t;

typedef struct benchmark_arg_s {
  workload_t workload;
  size_t thread_index;
  size_t slots_per_thread;
  pthread_barrier_t* ready_barrier;
  pthread_barrier_t* start_barrier;
  pthread_barrier_t* snapshot_barrier;
  pthread_barrier_t* free_barrier;
  pthread_barrier_t* freed_barrier;
  pthread_barrier_t* exit_barrier;
  const uint64_t* deadline_ns;
  remote_pair_t* remote_pair;
  uint64_t completed_operations;
  uint64_t checksum;
  uint64_t next_latency_sample;
  size_t latency_count;
} benchmark_arg_t;

typedef struct benchmark_result_s {
  double operations_per_second;
  double aggregate_nanoseconds_per_operation;
  uint64_t latency_p50_ns;
  uint64_t latency_p95_ns;
  uint64_t latency_p99_ns;
  uint64_t latency_p999_ns;
  uint64_t latency_max_ns;
  size_t latency_count;
  memory_stats_t before;
  memory_stats_t workload_end;
  memory_stats_t after_free;
  memory_stats_t after_purge;
  memory_stats_t after_purge_all;
  memory_stats_t after_thread_exit;
  long minor_faults;
  long major_faults;
  long voluntary_switches;
  long involuntary_switches;
  int purge_result;
  int purge_all_result;
  uint64_t purge_nanoseconds;
  uint64_t purge_all_nanoseconds;
  uint64_t checksum;
} benchmark_result_t;

static trace_entry_t traces[MAX_BENCH_THREADS][TRACE_LENGTH];
static void* live_slots[MAX_BENCH_THREADS][LIVE_SLOTS_TOTAL];
static uint64_t latency_samples[MAX_BENCH_THREADS][MAX_LATENCY_SAMPLES];
static uint64_t combined_latency_samples[MAX_BENCH_THREADS * MAX_LATENCY_SAMPLES];

static size_t system_page_size;
static uint64_t clock_overhead_ns;

static const char* const workload_names[WORKLOAD_COUNT] = {
    "local", "churn", "remote"};
static const char* const profile_names[PROFILE_COUNT] = {
    "small", "mixed", "page"};

static void fail_errno(const char* operation) {
  fprintf(stderr, "%s failed: %s\n", operation, strerror(errno));
  abort();
}

static void fail_pthread(const char* operation, int err) {
  fprintf(stderr, "%s failed: %d\n", operation, err);
  abort();
}

static uint64_t monotonic_nanoseconds(void) {
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) fail_errno("clock_gettime");
  return (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
}

static uint64_t next_random(uint64_t* state) {
  uint64_t x = *state;
  x ^= x << 13;
  x ^= x >> 7;
  x ^= x << 17;
  *state = x;
  return x;
}

static uint32_t profile_size(size_profile_t profile, uint64_t* random) {
  static const uint32_t small_sizes[] = {
      8, 16, 24, 32, 40, 48, 64, 80, 96, 128, 160, 192, 224, 256};
  static const uint32_t page_sizes[] = {
      4095, 4096, 4097, 8191, 8192, 8193, 16383, 16384,
      16385, 32767, 32768, 32769, 65535, 65536, 65537};

  uint64_t value = next_random(random);
  if (profile == PROFILE_SMALL) {
    return small_sizes[value % (sizeof(small_sizes) / sizeof(small_sizes[0]))];
  }
  if (profile == PROFILE_PAGE) {
    return page_sizes[value % (sizeof(page_sizes) / sizeof(page_sizes[0]))];
  }

  uint32_t bucket = (uint32_t)(value % 1000U);
  if (bucket < 700U) {
    return small_sizes[(value >> 16) % (sizeof(small_sizes) / sizeof(small_sizes[0]))];
  }
  if (bucket < 900U) {
    return 257U + (uint32_t)((value >> 16) % (4096U - 257U + 1U));
  }
  if (bucket < 990U) {
    return 4097U + (uint32_t)((value >> 16) % (65536U - 4097U + 1U));
  }
  return 65537U + (uint32_t)((value >> 16) % (262144U - 65537U + 1U));
}

static void prepare_traces(size_profile_t profile, size_t thread_count,
                           size_t slots_per_thread) {
  for (size_t thread = 0; thread < thread_count; thread++) {
    uint64_t random = 0x9e3779b97f4a7c15ULL ^ ((uint64_t)profile << 48) ^
                      ((uint64_t)thread + 1U) * 0xbf58476d1ce4e5b9ULL;
    for (size_t i = 0; i < TRACE_LENGTH; i++) {
      traces[thread][i].size = profile_size(profile, &random);
      traces[thread][i].slot =
          (uint32_t)(next_random(&random) % slots_per_thread);
    }
  }
}

__attribute__((noinline)) static uint64_t touch_allocation(void* pointer, size_t size,
                                                            uint64_t value) {
  volatile uint8_t* bytes = (volatile uint8_t*)pointer;
  uint8_t pattern = (uint8_t)(value ^ (value >> 8));
  for (size_t offset = 0; offset < size; offset += system_page_size) {
    bytes[offset] = (uint8_t)(pattern + offset);
  }
  bytes[size - 1U] = (uint8_t)(pattern ^ 0x5aU);
  return (uint64_t)bytes[0] + (uint64_t)bytes[size - 1U];
}

static void record_latency(benchmark_arg_t* arg, uint64_t start, uint64_t end) {
  if (arg->latency_count >= MAX_LATENCY_SAMPLES) return;
  uint64_t elapsed = end - start;
  latency_samples[arg->thread_index][arg->latency_count++] =
      elapsed > clock_overhead_ns ? elapsed - clock_overhead_ns : 0;
}

static bool should_sample_latency(benchmark_arg_t* arg, uint64_t completed) {
  if (completed != arg->next_latency_sample) return false;
  arg->next_latency_sample += LATENCY_SAMPLE_INTERVAL;
  return true;
}

static void initialize_live_set(benchmark_arg_t* arg) {
  void** slots = live_slots[arg->thread_index];
  for (size_t i = 0; i < arg->slots_per_thread; i++) {
    const trace_entry_t* entry = &traces[arg->thread_index][i & (TRACE_LENGTH - 1U)];
    slots[i] = malloc(entry->size);
    if (slots[i] == NULL) fail_errno("malloc");
    arg->checksum += touch_allocation(slots[i], entry->size, i + arg->thread_index);
  }
}

static void destroy_live_set(benchmark_arg_t* arg) {
  void** slots = live_slots[arg->thread_index];
  for (size_t i = 0; i < arg->slots_per_thread; i++) {
    free(slots[i]);
    slots[i] = NULL;
  }
}

static void wait_at_barrier(pthread_barrier_t* barrier) {
  int err = pthread_barrier_wait(barrier);
  if (err != 0 && err != PTHREAD_BARRIER_SERIAL_THREAD) {
    fail_pthread("pthread_barrier_wait", err);
  }
}

static void run_local(benchmark_arg_t* arg) {
  uint64_t completed = 0;
  size_t trace_index = 0;
  do {
    for (size_t i = 0; i < CLOCK_CHECK_INTERVAL; i++, completed++) {
      const trace_entry_t* entry =
          &traces[arg->thread_index][trace_index++ & (TRACE_LENGTH - 1U)];
      bool sample = should_sample_latency(arg, completed);
      uint64_t start = sample ? monotonic_nanoseconds() : 0;
      void* pointer = malloc(entry->size);
      if (pointer == NULL) fail_errno("malloc");
      arg->checksum += touch_allocation(pointer, entry->size, completed);
      free(pointer);
      if (sample) record_latency(arg, start, monotonic_nanoseconds());
    }
  } while (monotonic_nanoseconds() < *arg->deadline_ns);
  arg->completed_operations = completed;
}

static void run_churn(benchmark_arg_t* arg) {
  uint64_t completed = 0;
  size_t trace_index = arg->slots_per_thread;

  do {
    for (size_t i = 0; i < CLOCK_CHECK_INTERVAL; i++, completed++) {
      const trace_entry_t* entry =
          &traces[arg->thread_index][trace_index++ & (TRACE_LENGTH - 1U)];
      void** slot = &live_slots[arg->thread_index][entry->slot];
      bool sample = should_sample_latency(arg, completed);
      uint64_t start = sample ? monotonic_nanoseconds() : 0;
      free(*slot);
      *slot = malloc(entry->size);
      if (*slot == NULL) fail_errno("malloc");
      arg->checksum += touch_allocation(*slot, entry->size, completed);
      if (sample) record_latency(arg, start, monotonic_nanoseconds());
    }
  } while (monotonic_nanoseconds() < *arg->deadline_ns);

  arg->completed_operations = completed;
}

static void run_remote_producer(benchmark_arg_t* arg) {
  remote_pair_t* pair = arg->remote_pair;
  uint64_t completed = 0;
  size_t trace_index = 0;

  do {
    for (size_t i = 0; i < REMOTE_BATCH_SIZE; i++) {
      const trace_entry_t* entry =
          &traces[arg->thread_index][trace_index++ & (TRACE_LENGTH - 1U)];
      pair->pointers[i] = malloc(entry->size);
      if (pair->pointers[i] == NULL) fail_errno("malloc");
      arg->checksum +=
          touch_allocation(pair->pointers[i], entry->size, completed + i);
    }

    int err = pthread_mutex_lock(&pair->mutex);
    if (err != 0) fail_pthread("pthread_mutex_lock", err);
    pair->ready = true;
    err = pthread_cond_broadcast(&pair->condition);
    if (err != 0) fail_pthread("pthread_cond_broadcast", err);
    while (pair->ready) {
      err = pthread_cond_wait(&pair->condition, &pair->mutex);
      if (err != 0) fail_pthread("pthread_cond_wait", err);
    }
    err = pthread_mutex_unlock(&pair->mutex);
    if (err != 0) fail_pthread("pthread_mutex_unlock", err);
    completed += REMOTE_BATCH_SIZE;
  } while (monotonic_nanoseconds() < *arg->deadline_ns);

  int err = pthread_mutex_lock(&pair->mutex);
  if (err != 0) fail_pthread("pthread_mutex_lock", err);
  pair->done = true;
  err = pthread_cond_broadcast(&pair->condition);
  if (err != 0) fail_pthread("pthread_cond_broadcast", err);
  err = pthread_mutex_unlock(&pair->mutex);
  if (err != 0) fail_pthread("pthread_mutex_unlock", err);
  arg->completed_operations = completed;
}

static void run_remote_consumer(benchmark_arg_t* arg) {
  remote_pair_t* pair = arg->remote_pair;
  for (;;) {
    int err = pthread_mutex_lock(&pair->mutex);
    if (err != 0) fail_pthread("pthread_mutex_lock", err);
    while (!pair->ready && !pair->done) {
      err = pthread_cond_wait(&pair->condition, &pair->mutex);
      if (err != 0) fail_pthread("pthread_cond_wait", err);
    }
    if (pair->done && !pair->ready) {
      err = pthread_mutex_unlock(&pair->mutex);
      if (err != 0) fail_pthread("pthread_mutex_unlock", err);
      return;
    }
    err = pthread_mutex_unlock(&pair->mutex);
    if (err != 0) fail_pthread("pthread_mutex_unlock", err);

    for (size_t i = 0; i < REMOTE_BATCH_SIZE; i++) {
      free(pair->pointers[i]);
      pair->pointers[i] = NULL;
    }

    err = pthread_mutex_lock(&pair->mutex);
    if (err != 0) fail_pthread("pthread_mutex_lock", err);
    pair->ready = false;
    err = pthread_cond_broadcast(&pair->condition);
    if (err != 0) fail_pthread("pthread_cond_broadcast", err);
    err = pthread_mutex_unlock(&pair->mutex);
    if (err != 0) fail_pthread("pthread_mutex_unlock", err);
  }
}

static void* benchmark_worker(void* argument) {
  benchmark_arg_t* arg = (benchmark_arg_t*)argument;
  if (arg->workload == WORKLOAD_CHURN) {
    initialize_live_set(arg);
  }

  wait_at_barrier(arg->ready_barrier);
  wait_at_barrier(arg->start_barrier);
  if (arg->workload == WORKLOAD_CHURN) {
    run_churn(arg);
  } else if (arg->workload == WORKLOAD_LOCAL) {
    run_local(arg);
  } else if ((arg->thread_index & 1U) == 0) {
    run_remote_producer(arg);
  } else {
    run_remote_consumer(arg);
  }
  wait_at_barrier(arg->snapshot_barrier);
  wait_at_barrier(arg->free_barrier);
  if (arg->workload == WORKLOAD_CHURN) {
    destroy_live_set(arg);
  }
  wait_at_barrier(arg->freed_barrier);
  wait_at_barrier(arg->exit_barrier);
  return NULL;
}

static memory_stats_t read_memory_stats(void) {
  memory_stats_t stats = {0};
  FILE* file = fopen("/proc/self/smaps_rollup", "re");
  if (file == NULL) return stats;

  char line[256];
  while (fgets(line, sizeof(line), file) != NULL) {
    size_t value = 0;
    if (sscanf(line, "Rss: %zu kB", &value) == 1) stats.rss_kb = value;
    if (sscanf(line, "Pss: %zu kB", &value) == 1) stats.pss_kb = value;
    if (sscanf(line, "Private_Dirty: %zu kB", &value) == 1) {
      stats.private_dirty_kb = value;
    }
  }
  fclose(file);
  return stats;
}

static int compare_u64(const void* left, const void* right) {
  uint64_t a = *(const uint64_t*)left;
  uint64_t b = *(const uint64_t*)right;
  return a < b ? -1 : a > b ? 1 : 0;
}

static uint64_t percentile(const uint64_t* values, size_t count, size_t numerator,
                           size_t denominator) {
  if (count == 0) return 0;
  size_t index = (count * numerator + denominator - 1U) / denominator;
  if (index == 0) index = 1;
  if (index > count) index = count;
  return values[index - 1U];
}

static void calculate_latency(benchmark_result_t* result, benchmark_arg_t* args,
                              size_t thread_count) {
  size_t count = 0;
  for (size_t thread = 0; thread < thread_count; thread++) {
    size_t available = args[thread].latency_count;
    if (available > MAX_LATENCY_SAMPLES) available = MAX_LATENCY_SAMPLES;
    memcpy(&combined_latency_samples[count], latency_samples[thread],
           available * sizeof(uint64_t));
    count += available;
  }
  qsort(combined_latency_samples, count, sizeof(uint64_t), compare_u64);
  result->latency_count = count;
  result->latency_p50_ns = percentile(combined_latency_samples, count, 50, 100);
  result->latency_p95_ns = percentile(combined_latency_samples, count, 95, 100);
  result->latency_p99_ns = percentile(combined_latency_samples, count, 99, 100);
  result->latency_p999_ns = percentile(combined_latency_samples, count, 999, 1000);
  result->latency_max_ns = count == 0 ? 0 : combined_latency_samples[count - 1U];
}

static void initialize_remote_pairs(remote_pair_t* pairs, size_t pair_count) {
  for (size_t i = 0; i < pair_count; i++) {
    memset(&pairs[i], 0, sizeof(pairs[i]));
    int err = pthread_mutex_init(&pairs[i].mutex, NULL);
    if (err != 0) fail_pthread("pthread_mutex_init", err);
    err = pthread_cond_init(&pairs[i].condition, NULL);
    if (err != 0) fail_pthread("pthread_cond_init", err);
  }
}

static void destroy_remote_pairs(remote_pair_t* pairs, size_t pair_count) {
  for (size_t i = 0; i < pair_count; i++) {
    int err = pthread_cond_destroy(&pairs[i].condition);
    if (err != 0) fail_pthread("pthread_cond_destroy", err);
    err = pthread_mutex_destroy(&pairs[i].mutex);
    if (err != 0) fail_pthread("pthread_mutex_destroy", err);
  }
}

static benchmark_result_t run_benchmark(workload_t workload, size_profile_t profile,
                                        size_t thread_count, uint64_t duration_ns) {
  pthread_t threads[MAX_BENCH_THREADS];
  benchmark_arg_t args[MAX_BENCH_THREADS];
  remote_pair_t remote_pairs[MAX_REMOTE_PAIRS];
  pthread_barrier_t ready_barrier;
  pthread_barrier_t start_barrier;
  pthread_barrier_t snapshot_barrier;
  pthread_barrier_t free_barrier;
  pthread_barrier_t freed_barrier;
  pthread_barrier_t exit_barrier;
  uint64_t deadline_ns = 0;
  size_t slots_per_thread = LIVE_SLOTS_TOTAL / thread_count;

  prepare_traces(profile, thread_count, slots_per_thread);
  memset(args, 0, sizeof(args));
  memset(latency_samples, 0, sizeof(latency_samples));

  int err = pthread_barrier_init(&ready_barrier, NULL, (unsigned)thread_count + 1U);
  if (err != 0) fail_pthread("pthread_barrier_init", err);
  err = pthread_barrier_init(&start_barrier, NULL, (unsigned)thread_count + 1U);
  if (err != 0) fail_pthread("pthread_barrier_init", err);
  err = pthread_barrier_init(&snapshot_barrier, NULL, (unsigned)thread_count + 1U);
  if (err != 0) fail_pthread("pthread_barrier_init", err);
  err = pthread_barrier_init(&free_barrier, NULL, (unsigned)thread_count + 1U);
  if (err != 0) fail_pthread("pthread_barrier_init", err);
  err = pthread_barrier_init(&freed_barrier, NULL, (unsigned)thread_count + 1U);
  if (err != 0) fail_pthread("pthread_barrier_init", err);
  err = pthread_barrier_init(&exit_barrier, NULL, (unsigned)thread_count + 1U);
  if (err != 0) fail_pthread("pthread_barrier_init", err);

  size_t pair_count = workload == WORKLOAD_REMOTE ? thread_count / 2U : 0;
  initialize_remote_pairs(remote_pairs, pair_count);

  benchmark_result_t result = {0};
  result.before = read_memory_stats();
  struct rusage usage_before;
  struct rusage usage_after;
  if (getrusage(RUSAGE_SELF, &usage_before) != 0) fail_errno("getrusage");

  for (size_t i = 0; i < thread_count; i++) {
    args[i].workload = workload;
    args[i].thread_index = i;
    args[i].slots_per_thread = slots_per_thread;
    args[i].ready_barrier = &ready_barrier;
    args[i].start_barrier = &start_barrier;
    args[i].snapshot_barrier = &snapshot_barrier;
    args[i].free_barrier = &free_barrier;
    args[i].freed_barrier = &freed_barrier;
    args[i].exit_barrier = &exit_barrier;
    args[i].deadline_ns = &deadline_ns;
    args[i].remote_pair = workload == WORKLOAD_REMOTE ? &remote_pairs[i / 2U] : NULL;
    args[i].next_latency_sample = (i * 131U) % LATENCY_SAMPLE_INTERVAL;
    err = pthread_create(&threads[i], NULL, benchmark_worker, &args[i]);
    if (err != 0) fail_pthread("pthread_create", err);
  }

  wait_at_barrier(&ready_barrier);
  uint64_t start = monotonic_nanoseconds();
  deadline_ns = start + duration_ns;
  wait_at_barrier(&start_barrier);
  wait_at_barrier(&snapshot_barrier);
  uint64_t elapsed = monotonic_nanoseconds() - start;
  result.workload_end = read_memory_stats();
  wait_at_barrier(&free_barrier);
  wait_at_barrier(&freed_barrier);
  result.after_free = read_memory_stats();

  uint64_t purge_start = monotonic_nanoseconds();
  result.purge_result = mallopt(M_PURGE, 0);
  result.purge_nanoseconds = monotonic_nanoseconds() - purge_start;
  result.after_purge = read_memory_stats();

  purge_start = monotonic_nanoseconds();
  result.purge_all_result = mallopt(M_PURGE_ALL, 0);
  result.purge_all_nanoseconds = monotonic_nanoseconds() - purge_start;
  result.after_purge_all = read_memory_stats();
  wait_at_barrier(&exit_barrier);

  for (size_t i = 0; i < thread_count; i++) {
    err = pthread_join(threads[i], NULL);
    if (err != 0) fail_pthread("pthread_join", err);
  }
  result.after_thread_exit = read_memory_stats();
  if (getrusage(RUSAGE_SELF, &usage_after) != 0) fail_errno("getrusage");

  uint64_t operations = 0;
  for (size_t i = 0; i < thread_count; i++) {
    operations += args[i].completed_operations;
    result.checksum ^= args[i].checksum + i;
  }
  result.operations_per_second =
      (double)operations * 1000000000.0 / (double)elapsed;
  result.aggregate_nanoseconds_per_operation =
      operations == 0 ? 0 : (double)elapsed / (double)operations;
  calculate_latency(&result, args, thread_count);
  result.minor_faults = usage_after.ru_minflt - usage_before.ru_minflt;
  result.major_faults = usage_after.ru_majflt - usage_before.ru_majflt;
  result.voluntary_switches = usage_after.ru_nvcsw - usage_before.ru_nvcsw;
  result.involuntary_switches = usage_after.ru_nivcsw - usage_before.ru_nivcsw;

  destroy_remote_pairs(remote_pairs, pair_count);
  pthread_barrier_destroy(&exit_barrier);
  pthread_barrier_destroy(&freed_barrier);
  pthread_barrier_destroy(&free_barrier);
  pthread_barrier_destroy(&snapshot_barrier);
  pthread_barrier_destroy(&start_barrier);
  pthread_barrier_destroy(&ready_barrier);
  return result;
}

static void sort_results(benchmark_result_t* results, size_t count) {
  for (size_t i = 1; i < count; i++) {
    benchmark_result_t value = results[i];
    size_t j = i;
    while (j > 0 && results[j - 1U].aggregate_nanoseconds_per_operation >
                        value.aggregate_nanoseconds_per_operation) {
      results[j] = results[j - 1U];
      j--;
    }
    results[j] = value;
  }
}

static void print_results(workload_t workload, size_profile_t profile, size_t thread_count,
                          benchmark_result_t* results, size_t sample_count) {
  sort_results(results, sample_count);
  const benchmark_result_t* median = &results[sample_count / 2U];
  printf("workload=%-6s profile=%-5s threads=%2zu  %11.0f ops/s  "
         "%8.2f aggregate-ns/op  range %8.2f..%8.2f\n",
         workload_names[workload], profile_names[profile], thread_count,
         median->operations_per_second, median->aggregate_nanoseconds_per_operation,
         results[0].aggregate_nanoseconds_per_operation,
         results[sample_count - 1U].aggregate_nanoseconds_per_operation);

  if (median->latency_count != 0) {
    printf("  sampled latency ns: count=%zu p50=%llu p95=%llu p99=%llu "
           "p99.9=%llu max=%llu (clock baseline=%llu)\n",
           median->latency_count,
           (unsigned long long)median->latency_p50_ns,
           (unsigned long long)median->latency_p95_ns,
           (unsigned long long)median->latency_p99_ns,
           (unsigned long long)median->latency_p999_ns,
           (unsigned long long)median->latency_max_ns,
           (unsigned long long)clock_overhead_ns);
  } else {
    printf("  sampled latency: n/a for batched cross-thread handoff\n");
  }

  printf("  memory KiB rss/pss/private-dirty: before=%zu/%zu/%zu "
         "workload-end=%zu/%zu/%zu after-free=%zu/%zu/%zu\n",
         median->before.rss_kb, median->before.pss_kb, median->before.private_dirty_kb,
         median->workload_end.rss_kb, median->workload_end.pss_kb,
         median->workload_end.private_dirty_kb,
         median->after_free.rss_kb, median->after_free.pss_kb,
         median->after_free.private_dirty_kb);
  printf("  purge: M_PURGE result=%d time=%llu ns rss/pss/private-dirty=%zu/%zu/%zu; "
         "M_PURGE_ALL result=%d time=%llu ns rss/pss/private-dirty=%zu/%zu/%zu\n",
         median->purge_result, (unsigned long long)median->purge_nanoseconds,
         median->after_purge.rss_kb, median->after_purge.pss_kb,
         median->after_purge.private_dirty_kb, median->purge_all_result,
         (unsigned long long)median->purge_all_nanoseconds,
         median->after_purge_all.rss_kb, median->after_purge_all.pss_kb,
         median->after_purge_all.private_dirty_kb);
  printf("  after-thread-exit rss/pss/private-dirty=%zu/%zu/%zu\n",
         median->after_thread_exit.rss_kb, median->after_thread_exit.pss_kb,
         median->after_thread_exit.private_dirty_kb);
  printf("  events: minflt=%ld majflt=%ld voluntary-csw=%ld involuntary-csw=%ld "
         "checksum=%llu\n",
         median->minor_faults, median->major_faults, median->voluntary_switches,
         median->involuntary_switches, (unsigned long long)median->checksum);
}

static uint64_t calibrate_clock_overhead(void) {
  enum { CALIBRATION_SAMPLES = 10001 };
  static uint64_t values[CALIBRATION_SAMPLES];
  for (size_t i = 0; i < CALIBRATION_SAMPLES; i++) {
    uint64_t start = monotonic_nanoseconds();
    values[i] = monotonic_nanoseconds() - start;
  }
  qsort(values, CALIBRATION_SAMPLES, sizeof(uint64_t), compare_u64);
  return values[CALIBRATION_SAMPLES / 2U];
}

int main(void) {
  long page_size = sysconf(_SC_PAGESIZE);
  long online_cpus = sysconf(_SC_NPROCESSORS_ONLN);
  if (page_size <= 0) fail_errno("sysconf(_SC_PAGESIZE)");
  system_page_size = (size_t)page_size;
  clock_overhead_ns = calibrate_clock_overhead();

  static const size_t thread_counts[] = {1, 2, 4, 8, 16};
  printf("allocator=system-libc online_cpus=%ld page_size=%zu samples=%u "
         "sample_duration=%.2fs warmup=%.2fs\n",
         online_cpus, system_page_size, BENCHMARK_SAMPLES,
         (double)BENCHMARK_DURATION_NS / 1000000000.0,
         (double)WARMUP_DURATION_NS / 1000000000.0);
  printf("note: aggregate-ns/op is the inverse of aggregate throughput, not "
         "single-operation latency\n");
  printf("note: workload-end contains a live set only for churn; local and remote "
         "have already freed their user allocations\n");

  for (size_t workload = 0; workload < WORKLOAD_COUNT; workload++) {
    for (size_t profile = 0; profile < PROFILE_COUNT; profile++) {
      for (size_t thread_index = 0;
           thread_index < sizeof(thread_counts) / sizeof(thread_counts[0]);
           thread_index++) {
        size_t thread_count = thread_counts[thread_index];
        if (workload == WORKLOAD_REMOTE && thread_count < 2U) continue;

        (void)run_benchmark((workload_t)workload, (size_profile_t)profile,
                            thread_count, WARMUP_DURATION_NS);
        benchmark_result_t results[BENCHMARK_SAMPLES];
        for (size_t sample = 0; sample < BENCHMARK_SAMPLES; sample++) {
          results[sample] =
              run_benchmark((workload_t)workload, (size_profile_t)profile,
                            thread_count, BENCHMARK_DURATION_NS);
        }
        print_results((workload_t)workload, (size_profile_t)profile, thread_count,
                      results, BENCHMARK_SAMPLES);
      }
    }
  }
  return 0;
}
