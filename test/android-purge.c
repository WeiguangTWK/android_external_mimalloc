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

#include "mimalloc.h"
#include "mimalloc/types.h"
#include "mimalloc_adapt.h"

#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define PURGE_WORKER_COUNT 4
#define PURGE_ALLOCATION_COUNT 64
#define PURGE_STRESS_WORKER_COUNT 8
#define PURGE_STRESS_PURGER_COUNT 2
#define PURGE_STRESS_SLOT_COUNT 32
#define PURGE_STRESS_ROUNDS 100

#define TEST_CHECK(expression)                                                             \
  do {                                                                                     \
    if (!(expression)) {                                                                   \
      fprintf(stderr, "android purge test failed: %s (%s:%d)\n", #expression, __FILE__, \
              __LINE__);                                                                   \
      abort();                                                                             \
    }                                                                                      \
  } while (0)

typedef struct purge_test_state_s {
  pthread_barrier_t allocated;
  pthread_barrier_t release;
  mi_heap_t* heaps[PURGE_WORKER_COUNT];
  void* pointers[PURGE_WORKER_COUNT][PURGE_ALLOCATION_COUNT];
} purge_test_state_t;

typedef struct purge_worker_arg_s {
  purge_test_state_t* state;
  size_t worker_index;
} purge_worker_arg_t;

typedef struct purge_stress_state_s {
  pthread_barrier_t start;
  _Atomic(size_t) ready_workers;
  _Atomic(size_t) completed_purges;
  _Atomic(size_t) allocation_operations;
  _Atomic(bool) stop;
} purge_stress_state_t;

typedef struct purge_stress_worker_arg_s {
  purge_stress_state_t* state;
  size_t worker_index;
} purge_stress_worker_arg_t;

static void wait_at_barrier(pthread_barrier_t* barrier) {
  int err = pthread_barrier_wait(barrier);
  TEST_CHECK(err == 0 || err == PTHREAD_BARRIER_SERIAL_THREAD);
}

static void* purge_worker(void* argument) {
  purge_worker_arg_t* arg = (purge_worker_arg_t*)argument;
  const size_t worker = arg->worker_index;

  for (size_t i = 0; i < PURGE_ALLOCATION_COUNT; i++) {
    size_t size = 512 + ((worker + 1) * (i + 3) * 37) % 4096;
    arg->state->pointers[worker][i] = mimalloc_malloc(size);
    TEST_CHECK(arg->state->pointers[worker][i] != NULL);
  }
  arg->state->heaps[worker] = mi_heap_get_default();

  wait_at_barrier(&arg->state->allocated);
  wait_at_barrier(&arg->state->release);

  // The heap remains usable after another thread collected all of its pages.
  void* probe = mimalloc_malloc(128 + worker);
  TEST_CHECK(probe != NULL);
  mimalloc_free(probe);
  return NULL;
}

static uint32_t next_random(uint32_t* state) {
  uint32_t value = *state;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  *state = value;
  return value;
}

static void* purge_stress_worker(void* argument) {
  purge_stress_worker_arg_t* arg = (purge_stress_worker_arg_t*)argument;
  purge_stress_state_t* state = arg->state;
  void* slots[PURGE_STRESS_SLOT_COUNT] = {};
  uint32_t random = 0x9e3779b9U ^ (uint32_t)(arg->worker_index * 0x85ebca6bU);
  size_t operations = 0;

  wait_at_barrier(&state->start);
  for (size_t i = 0; i < PURGE_STRESS_SLOT_COUNT; i++) {
    size_t size = 1 + next_random(&random) % 16384;
    slots[i] = mimalloc_malloc(size);
    TEST_CHECK(slots[i] != NULL);
    operations++;
  }
  atomic_fetch_add_explicit(&state->ready_workers, 1, memory_order_release);

  while (!atomic_load_explicit(&state->stop, memory_order_acquire)) {
    size_t slot = next_random(&random) % PURGE_STRESS_SLOT_COUNT;
    mimalloc_free(slots[slot]);
    size_t size = 1 + next_random(&random) % 16384;
    slots[slot] = mimalloc_malloc(size);
    TEST_CHECK(slots[slot] != NULL);
    operations += 2;
  }

  for (size_t i = 0; i < PURGE_STRESS_SLOT_COUNT; i++) {
    mimalloc_free(slots[i]);
  }
  atomic_fetch_add_explicit(&state->allocation_operations, operations, memory_order_relaxed);
  return NULL;
}

static void* purge_stress_purger(void* argument) {
  purge_stress_state_t* state = (purge_stress_state_t*)argument;
  wait_at_barrier(&state->start);
  while (atomic_load_explicit(&state->ready_workers, memory_order_acquire) !=
         PURGE_STRESS_WORKER_COUNT) {
    sched_yield();
  }

  for (size_t i = 0; i < PURGE_STRESS_ROUNDS; i++) {
    TEST_CHECK(mimalloc_mallopt(M_PURGE_ALL, 0) == 1);
    atomic_fetch_add_explicit(&state->completed_purges, 1, memory_order_relaxed);
  }
  return NULL;
}

static void run_concurrent_purge_stress(void) {
  purge_stress_state_t state = {};
  pthread_t workers[PURGE_STRESS_WORKER_COUNT];
  pthread_t purgers[PURGE_STRESS_PURGER_COUNT];
  purge_stress_worker_arg_t worker_args[PURGE_STRESS_WORKER_COUNT];
  const size_t participant_count =
      PURGE_STRESS_WORKER_COUNT + PURGE_STRESS_PURGER_COUNT + 1;

  TEST_CHECK(pthread_barrier_init(&state.start, NULL, participant_count) == 0);
  for (size_t i = 0; i < PURGE_STRESS_WORKER_COUNT; i++) {
    worker_args[i].state = &state;
    worker_args[i].worker_index = i;
    TEST_CHECK(pthread_create(&workers[i], NULL, purge_stress_worker, &worker_args[i]) == 0);
  }
  for (size_t i = 0; i < PURGE_STRESS_PURGER_COUNT; i++) {
    TEST_CHECK(pthread_create(&purgers[i], NULL, purge_stress_purger, &state) == 0);
  }

  wait_at_barrier(&state.start);
  for (size_t i = 0; i < PURGE_STRESS_PURGER_COUNT; i++) {
    TEST_CHECK(pthread_join(purgers[i], NULL) == 0);
  }
  atomic_store_explicit(&state.stop, true, memory_order_release);
  for (size_t i = 0; i < PURGE_STRESS_WORKER_COUNT; i++) {
    TEST_CHECK(pthread_join(workers[i], NULL) == 0);
  }

  TEST_CHECK(atomic_load_explicit(&state.completed_purges, memory_order_relaxed) ==
             PURGE_STRESS_PURGER_COUNT * PURGE_STRESS_ROUNDS);
  TEST_CHECK(atomic_load_explicit(&state.allocation_operations, memory_order_relaxed) >=
             PURGE_STRESS_WORKER_COUNT * PURGE_STRESS_SLOT_COUNT);
  TEST_CHECK(pthread_barrier_destroy(&state.start) == 0);
}

int main(void) {
  purge_test_state_t state = {};
  pthread_t workers[PURGE_WORKER_COUNT];
  purge_worker_arg_t args[PURGE_WORKER_COUNT];

  TEST_CHECK(pthread_barrier_init(&state.allocated, NULL, PURGE_WORKER_COUNT + 1) == 0);
  TEST_CHECK(pthread_barrier_init(&state.release, NULL, PURGE_WORKER_COUNT + 1) == 0);

  for (size_t i = 0; i < PURGE_WORKER_COUNT; i++) {
    args[i].state = &state;
    args[i].worker_index = i;
    TEST_CHECK(pthread_create(&workers[i], NULL, purge_worker, &args[i]) == 0);
  }

  wait_at_barrier(&state.allocated);
  for (size_t worker = 0; worker < PURGE_WORKER_COUNT; worker++) {
    TEST_CHECK(state.heaps[worker] != NULL);
    TEST_CHECK(state.heaps[worker]->page_count > 0);
    for (size_t i = 0; i < PURGE_ALLOCATION_COUNT; i++) {
      mimalloc_free(state.pointers[worker][i]);
    }
  }

  // A normal purge only collects the calling thread's default heap.
  TEST_CHECK(mimalloc_mallopt(M_PURGE, 0) == 1);
  for (size_t i = 0; i < PURGE_WORKER_COUNT; i++) {
    TEST_CHECK(state.heaps[i]->page_count > 0);
  }

  TEST_CHECK(mimalloc_mallopt(M_PURGE_ALL, 0) == 1);
  for (size_t i = 0; i < PURGE_WORKER_COUNT; i++) {
    TEST_CHECK(state.heaps[i]->page_count == 0);
  }

  wait_at_barrier(&state.release);
  for (size_t i = 0; i < PURGE_WORKER_COUNT; i++) {
    TEST_CHECK(pthread_join(workers[i], NULL) == 0);
  }

  TEST_CHECK(pthread_barrier_destroy(&state.release) == 0);
  TEST_CHECK(pthread_barrier_destroy(&state.allocated) == 0);
  run_concurrent_purge_stress();
  return 0;
}
