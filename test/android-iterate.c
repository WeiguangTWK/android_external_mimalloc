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

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define ITERATE_WORKER_COUNT 6
#define ITERATE_ALLOCATION_COUNT 16
#define ITERATE_POINTER_COUNT (ITERATE_WORKER_COUNT * ITERATE_ALLOCATION_COUNT)

#define TEST_CHECK(expression)                                                               \
  do {                                                                                       \
    if (!(expression)) {                                                                     \
      fprintf(stderr, "android iterate test failed: %s (%s:%d)\n", #expression, __FILE__,   \
              __LINE__);                                                                     \
      abort();                                                                               \
    }                                                                                        \
  } while (0)

typedef struct iterate_test_state_s {
  pthread_barrier_t allocated;
  pthread_barrier_t release;
  void* pointers[ITERATE_POINTER_COUNT];
  size_t visits[ITERATE_POINTER_COUNT];
} iterate_test_state_t;

typedef struct iterate_worker_arg_s {
  iterate_test_state_t* state;
  size_t worker_index;
} iterate_worker_arg_t;

static void wait_at_barrier(pthread_barrier_t* barrier) {
  int err = pthread_barrier_wait(barrier);
  TEST_CHECK(err == 0 || err == PTHREAD_BARRIER_SERIAL_THREAD);
}

static void* allocation_worker(void* argument) {
  iterate_worker_arg_t* arg = (iterate_worker_arg_t*)argument;
  size_t first = arg->worker_index * ITERATE_ALLOCATION_COUNT;

  for (size_t i = 0; i < ITERATE_ALLOCATION_COUNT; i++) {
    size_t pointer_index = first + i;
    size_t size = 1 + ((arg->worker_index + 1) * (i + 17) * 37) % 8192;
    arg->state->pointers[pointer_index] = mimalloc_malloc(size);
    TEST_CHECK(arg->state->pointers[pointer_index] != NULL);
  }

  wait_at_barrier(&arg->state->allocated);
  wait_at_barrier(&arg->state->release);

  for (size_t i = 0; i < ITERATE_ALLOCATION_COUNT; i++) {
    mimalloc_free(arg->state->pointers[first + i]);
  }
  return NULL;
}

static void record_known_pointer(uintptr_t base, size_t size, void* argument) {
  (void)size;
  iterate_test_state_t* state = (iterate_test_state_t*)argument;
  for (size_t i = 0; i < ITERATE_POINTER_COUNT; i++) {
    if ((uintptr_t)state->pointers[i] == base) {
      state->visits[i]++;
      return;
    }
  }
}

int main(void) {
  iterate_test_state_t state = {};
  pthread_t workers[ITERATE_WORKER_COUNT];
  iterate_worker_arg_t args[ITERATE_WORKER_COUNT];

  TEST_CHECK(pthread_barrier_init(&state.allocated, NULL, ITERATE_WORKER_COUNT + 1) == 0);
  TEST_CHECK(pthread_barrier_init(&state.release, NULL, ITERATE_WORKER_COUNT + 1) == 0);

  for (size_t i = 0; i < ITERATE_WORKER_COUNT; i++) {
    args[i].state = &state;
    args[i].worker_index = i;
    TEST_CHECK(pthread_create(&workers[i], NULL, allocation_worker, &args[i]) == 0);
  }

  wait_at_barrier(&state.allocated);
  mimalloc_malloc_disable();
  int iterate_result =
      mimalloc_malloc_iterate(0, UINTPTR_MAX, record_known_pointer, &state);
  mimalloc_malloc_enable();

  TEST_CHECK(iterate_result == 0);
  for (size_t i = 0; i < ITERATE_POINTER_COUNT; i++) {
    TEST_CHECK(state.visits[i] == 1);
  }

  wait_at_barrier(&state.release);
  for (size_t i = 0; i < ITERATE_WORKER_COUNT; i++) {
    TEST_CHECK(pthread_join(workers[i], NULL) == 0);
  }

  TEST_CHECK(pthread_barrier_destroy(&state.release) == 0);
  TEST_CHECK(pthread_barrier_destroy(&state.allocated) == 0);
  return 0;
}
