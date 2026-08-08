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

#include "mimalloc_adapt.h"

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define MALLINFO_WORKER_COUNT 4
#define MALLINFO_ALLOCATION_COUNT 32
#define MALLINFO_STRESS_WORKER_COUNT 6
#define MALLINFO_STRESS_READER_COUNT 2
#define MALLINFO_STRESS_SLOT_COUNT 16
#define MALLINFO_STRESS_READS 100
#define MALLINFO_LARGE_COUNT 3
#define MALLINFO_LARGE_SIZE ((size_t)768 * 1024 * 1024)
#define MALLINFO_PUBLIC_ALLOCATION_COUNT 64
#define MALLINFO_TIMING_CALLS 10000

#define TEST_CHECK(expression)                                                                \
  do {                                                                                        \
    if (!(expression)) {                                                                      \
      fprintf(stderr, "android mallinfo test failed: %s (%s:%d)\n", #expression, __FILE__, \
              __LINE__);                                                                      \
      abort();                                                                                \
    }                                                                                         \
  } while (0)

typedef struct mallinfo_test_state_s {
  pthread_barrier_t allocated;
  pthread_barrier_t release;
  void* pointers[MALLINFO_WORKER_COUNT][MALLINFO_ALLOCATION_COUNT];
  size_t usable[MALLINFO_WORKER_COUNT];
} mallinfo_test_state_t;

typedef struct mallinfo_worker_arg_s {
  mallinfo_test_state_t* state;
  size_t worker_index;
} mallinfo_worker_arg_t;

typedef struct mallinfo_stress_state_s {
  pthread_barrier_t start;
  _Atomic(size_t) ready_workers;
  _Atomic(size_t) completed_reads;
  _Atomic(bool) stop;
} mallinfo_stress_state_t;

typedef struct mallinfo_stress_worker_arg_s {
  mallinfo_stress_state_t* state;
  size_t worker_index;
} mallinfo_stress_worker_arg_t;

static void wait_at_barrier(pthread_barrier_t* barrier) {
  int err = pthread_barrier_wait(barrier);
  TEST_CHECK(err == 0 || err == PTHREAD_BARRIER_SERIAL_THREAD);
}

static void check_mapping_fields(struct mallinfo info) {
  TEST_CHECK(info.hblkhd == info.usmblks);
  TEST_CHECK(info.hblkhd >= info.uordblks);
  TEST_CHECK(info.fsmblks == info.fordblks);
  TEST_CHECK(info.hblkhd - info.uordblks == info.fordblks);
}

static void print_public_mallinfo(const char* label, struct mallinfo info) {
  printf("public %-9s arena=%zu hblkhd=%zu usmblks=%zu uordblks=%zu "
         "fordblks=%zu keepcost=%zu\n",
         label, info.arena, info.hblkhd, info.usmblks, info.uordblks,
         info.fordblks, info.keepcost);
}

static void check_mallinfo2_matches(struct mallinfo info, struct mallinfo2 info2) {
  TEST_CHECK(info.arena == info2.arena);
  TEST_CHECK(info.ordblks == info2.ordblks);
  TEST_CHECK(info.smblks == info2.smblks);
  TEST_CHECK(info.hblks == info2.hblks);
  TEST_CHECK(info.hblkhd == info2.hblkhd);
  TEST_CHECK(info.usmblks == info2.usmblks);
  TEST_CHECK(info.fsmblks == info2.fsmblks);
  TEST_CHECK(info.uordblks == info2.uordblks);
  TEST_CHECK(info.fordblks == info2.fordblks);
  TEST_CHECK(info.keepcost == info2.keepcost);
}

static void test_bionic_public_entrypoint(void) {
  void* pointers[MALLINFO_PUBLIC_ALLOCATION_COUNT] = {};
  size_t expected = 0;
  struct mallinfo before = mallinfo();
  struct mallinfo2 before2 = mallinfo2();
  check_mallinfo2_matches(before, before2);

  for (size_t i = 0; i < MALLINFO_PUBLIC_ALLOCATION_COUNT; i++) {
    const size_t size = 4096 + (i + 1) * 8191;
    pointers[i] = malloc(size);
    TEST_CHECK(pointers[i] != NULL);
    memset(pointers[i], (int)i, size);
    expected += size;
  }

  struct mallinfo allocated = mallinfo();
  struct mallinfo2 allocated2 = mallinfo2();
  check_mallinfo2_matches(allocated, allocated2);
  for (size_t i = 0; i < MALLINFO_PUBLIC_ALLOCATION_COUNT; i++) {
    free(pointers[i]);
  }
  struct mallinfo freed = mallinfo();
  struct mallinfo2 freed2 = mallinfo2();
  check_mallinfo2_matches(freed, freed2);

  print_public_mallinfo("before", before);
  print_public_mallinfo("allocated", allocated);
  print_public_mallinfo("freed", freed);
  printf("public minimum requested increase=%zu actual=%zu; "
         "minimum requested free decrease=%zu actual=%zu\n",
         expected,
         allocated.uordblks >= before.uordblks
             ? allocated.uordblks - before.uordblks
             : 0,
         expected,
         allocated.uordblks >= freed.uordblks
             ? allocated.uordblks - freed.uordblks
             : 0);

  check_mapping_fields(allocated);
  check_mapping_fields(freed);
  TEST_CHECK(allocated.uordblks >= before.uordblks + expected);
  TEST_CHECK(freed.uordblks + expected <= allocated.uordblks);
}

static void test_adapter_entrypoints(void) {
  struct mallinfo before = mimalloc_mallinfo();
  void* calloc_pointer = mimalloc_calloc(32, 1024);
  void* aligned_pointer = mimalloc_aligned_alloc(4096, 8192);
  void* memalign_pointer = mimalloc_memalign(2048, 8193);
  void* posix_pointer = NULL;
  TEST_CHECK(calloc_pointer != NULL);
  TEST_CHECK(aligned_pointer != NULL);
  TEST_CHECK(memalign_pointer != NULL);
  TEST_CHECK(mimalloc_posix_memalign(&posix_pointer, 1024, 16385) == 0);
  TEST_CHECK(posix_pointer != NULL);

  calloc_pointer = mimalloc_realloc(calloc_pointer, 128 * 1024);
  TEST_CHECK(calloc_pointer != NULL);
  struct mallinfo allocated = mimalloc_mallinfo();
  TEST_CHECK(allocated.uordblks > before.uordblks);

  void* failed = mimalloc_realloc(calloc_pointer, SIZE_MAX);
  TEST_CHECK(failed == NULL);
  TEST_CHECK(mimalloc_mallinfo().uordblks == allocated.uordblks);

  mimalloc_free(calloc_pointer);
  mimalloc_free(aligned_pointer);
  mimalloc_free(memalign_pointer);
  mimalloc_free(posix_pointer);
  TEST_CHECK(mimalloc_mallinfo().uordblks == before.uordblks);
}

static void test_fork_preserves_accounting(void) {
  struct mallinfo before = mimalloc_mallinfo();
  void* pointer = mimalloc_malloc(256 * 1024);
  TEST_CHECK(pointer != NULL);
  const size_t usable = mimalloc_malloc_usable_size(pointer);
  struct mallinfo allocated = mimalloc_mallinfo();
  TEST_CHECK(allocated.uordblks >= before.uordblks + usable);

  pid_t pid = fork();
  TEST_CHECK(pid >= 0);
  if (pid == 0) {
    struct mallinfo inherited = mimalloc_mallinfo();
    TEST_CHECK(inherited.uordblks >= before.uordblks + usable);
    mimalloc_free(pointer);
    struct mallinfo freed = mimalloc_mallinfo();
    TEST_CHECK(freed.uordblks + usable <= inherited.uordblks);
    _exit(0);
  }

  int status = 0;
  TEST_CHECK(waitpid(pid, &status, 0) == pid);
  TEST_CHECK(WIFEXITED(status));
  TEST_CHECK(WEXITSTATUS(status) == 0);
  mimalloc_free(pointer);
  TEST_CHECK(mimalloc_mallinfo().uordblks == before.uordblks);
}

static uint64_t monotonic_nanoseconds(void) {
  struct timespec now;
  TEST_CHECK(clock_gettime(CLOCK_MONOTONIC, &now) == 0);
  return (uint64_t)now.tv_sec * 1000000000ULL + (uint64_t)now.tv_nsec;
}

static void measure_mallinfo_latency(void) {
  size_t checksum = 0;
  const uint64_t start = monotonic_nanoseconds();
  for (size_t i = 0; i < MALLINFO_TIMING_CALLS; i++) {
    checksum += mimalloc_mallinfo().uordblks;
  }
  const uint64_t elapsed = monotonic_nanoseconds() - start;
  printf("mallinfo hot-path calls=%u average=%.2f ns/call checksum=%zu\n",
         MALLINFO_TIMING_CALLS, (double)elapsed / MALLINFO_TIMING_CALLS, checksum);
}

static void* allocation_worker(void* argument) {
  mallinfo_worker_arg_t* arg = (mallinfo_worker_arg_t*)argument;
  const size_t worker = arg->worker_index;
  size_t usable = 0;

  for (size_t i = 0; i < MALLINFO_ALLOCATION_COUNT; i++) {
    size_t size = 1 + ((worker + 3) * (i + 11) * 53) % 32768;
    void* pointer = mimalloc_malloc(size);
    TEST_CHECK(pointer != NULL);
    arg->state->pointers[worker][i] = pointer;
    usable += mimalloc_malloc_usable_size(pointer);
  }
  arg->state->usable[worker] = usable;

  wait_at_barrier(&arg->state->allocated);
  wait_at_barrier(&arg->state->release);
  return NULL;
}

static void test_live_thread_allocations(void) {
  mallinfo_test_state_t state = {};
  pthread_t workers[MALLINFO_WORKER_COUNT];
  mallinfo_worker_arg_t args[MALLINFO_WORKER_COUNT];
  struct mallinfo before = mimalloc_mallinfo();

  TEST_CHECK(pthread_barrier_init(&state.allocated, NULL, MALLINFO_WORKER_COUNT + 1) == 0);
  TEST_CHECK(pthread_barrier_init(&state.release, NULL, MALLINFO_WORKER_COUNT + 1) == 0);
  for (size_t i = 0; i < MALLINFO_WORKER_COUNT; i++) {
    args[i].state = &state;
    args[i].worker_index = i;
    TEST_CHECK(pthread_create(&workers[i], NULL, allocation_worker, &args[i]) == 0);
  }

  wait_at_barrier(&state.allocated);
  size_t expected = 0;
  for (size_t worker = 0; worker < MALLINFO_WORKER_COUNT; worker++) {
    expected += state.usable[worker];
  }

  struct mallinfo allocated = mimalloc_mallinfo();
  check_mapping_fields(allocated);
  TEST_CHECK(allocated.uordblks >= before.uordblks + expected);

  for (size_t worker = 0; worker < MALLINFO_WORKER_COUNT; worker++) {
    for (size_t i = 0; i < MALLINFO_ALLOCATION_COUNT; i++) {
      mimalloc_free(state.pointers[worker][i]);
    }
  }
  struct mallinfo freed = mimalloc_mallinfo();
  check_mapping_fields(freed);
  if (freed.uordblks + expected > allocated.uordblks) {
    fprintf(stderr,
            "mallinfo cross-thread free mismatch: before=%zu allocated=%zu "
            "freed=%zu expected_drop=%zu actual_drop=%zu\n",
            before.uordblks, allocated.uordblks, freed.uordblks, expected,
            allocated.uordblks >= freed.uordblks
                ? allocated.uordblks - freed.uordblks
                : 0);
  }
  TEST_CHECK(freed.uordblks + expected <= allocated.uordblks);

  wait_at_barrier(&state.release);
  for (size_t i = 0; i < MALLINFO_WORKER_COUNT; i++) {
    TEST_CHECK(pthread_join(workers[i], NULL) == 0);
  }
  TEST_CHECK(pthread_barrier_destroy(&state.release) == 0);
  TEST_CHECK(pthread_barrier_destroy(&state.allocated) == 0);
}

static void test_larger_than_int_max(void) {
#if UINTPTR_MAX > UINT32_MAX
  void* pointers[MALLINFO_LARGE_COUNT] = {};
  for (size_t i = 0; i < MALLINFO_LARGE_COUNT; i++) {
    pointers[i] = mimalloc_malloc(MALLINFO_LARGE_SIZE);
    TEST_CHECK(pointers[i] != NULL);
  }
  struct mallinfo info = mimalloc_mallinfo();
  check_mapping_fields(info);
  TEST_CHECK(info.uordblks > (size_t)INT_MAX);
  for (size_t i = 0; i < MALLINFO_LARGE_COUNT; i++) {
    mimalloc_free(pointers[i]);
  }
#endif
}

static uint32_t next_random(uint32_t* state) {
  uint32_t value = *state;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  *state = value;
  return value;
}

static void* mallinfo_stress_worker(void* argument) {
  mallinfo_stress_worker_arg_t* arg = (mallinfo_stress_worker_arg_t*)argument;
  mallinfo_stress_state_t* state = arg->state;
  void* slots[MALLINFO_STRESS_SLOT_COUNT] = {};
  uint32_t random = 0x7f4a7c15U ^ (uint32_t)(arg->worker_index * 0x9e3779b9U);

  wait_at_barrier(&state->start);
  for (size_t i = 0; i < MALLINFO_STRESS_SLOT_COUNT; i++) {
    slots[i] = mimalloc_malloc(1 + next_random(&random) % 65536);
    TEST_CHECK(slots[i] != NULL);
  }
  atomic_fetch_add_explicit(&state->ready_workers, 1, memory_order_release);

  while (!atomic_load_explicit(&state->stop, memory_order_acquire)) {
    size_t slot = next_random(&random) % MALLINFO_STRESS_SLOT_COUNT;
    mimalloc_free(slots[slot]);
    slots[slot] = mimalloc_malloc(1 + next_random(&random) % 65536);
    TEST_CHECK(slots[slot] != NULL);
  }
  for (size_t i = 0; i < MALLINFO_STRESS_SLOT_COUNT; i++) {
    mimalloc_free(slots[i]);
  }
  return NULL;
}

static void* mallinfo_stress_reader(void* argument) {
  mallinfo_stress_state_t* state = (mallinfo_stress_state_t*)argument;
  wait_at_barrier(&state->start);
  while (atomic_load_explicit(&state->ready_workers, memory_order_acquire) !=
         MALLINFO_STRESS_WORKER_COUNT) {
    sched_yield();
  }
  for (size_t i = 0; i < MALLINFO_STRESS_READS; i++) {
    check_mapping_fields(mimalloc_mallinfo());
    atomic_fetch_add_explicit(&state->completed_reads, 1, memory_order_relaxed);
  }
  return NULL;
}

static void test_concurrent_snapshots(void) {
  mallinfo_stress_state_t state = {};
  pthread_t workers[MALLINFO_STRESS_WORKER_COUNT];
  pthread_t readers[MALLINFO_STRESS_READER_COUNT];
  mallinfo_stress_worker_arg_t args[MALLINFO_STRESS_WORKER_COUNT];
  const size_t participants =
      MALLINFO_STRESS_WORKER_COUNT + MALLINFO_STRESS_READER_COUNT + 1;

  TEST_CHECK(pthread_barrier_init(&state.start, NULL, participants) == 0);
  for (size_t i = 0; i < MALLINFO_STRESS_WORKER_COUNT; i++) {
    args[i].state = &state;
    args[i].worker_index = i;
    TEST_CHECK(pthread_create(&workers[i], NULL, mallinfo_stress_worker, &args[i]) == 0);
  }
  for (size_t i = 0; i < MALLINFO_STRESS_READER_COUNT; i++) {
    TEST_CHECK(pthread_create(&readers[i], NULL, mallinfo_stress_reader, &state) == 0);
  }

  wait_at_barrier(&state.start);
  for (size_t i = 0; i < MALLINFO_STRESS_READER_COUNT; i++) {
    TEST_CHECK(pthread_join(readers[i], NULL) == 0);
  }
  atomic_store_explicit(&state.stop, true, memory_order_release);
  for (size_t i = 0; i < MALLINFO_STRESS_WORKER_COUNT; i++) {
    TEST_CHECK(pthread_join(workers[i], NULL) == 0);
  }

  TEST_CHECK(atomic_load_explicit(&state.completed_reads, memory_order_relaxed) ==
             MALLINFO_STRESS_READER_COUNT * MALLINFO_STRESS_READS);
  TEST_CHECK(pthread_barrier_destroy(&state.start) == 0);
}

int main(void) {
  test_adapter_entrypoints();
  test_fork_preserves_accounting();
  test_live_thread_allocations();
  test_larger_than_int_max();
  test_concurrent_snapshots();
  test_bionic_public_entrypoint();
  measure_mallinfo_latency();
  return 0;
}
