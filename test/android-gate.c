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
#include "mimalloc_android_gate.h"

#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/wait.h>
#include <unistd.h>

#define WORKER_COUNT 8
#define DISABLE_ITERATIONS 1000

#define TEST_CHECK(expression)                                                               \
  do {                                                                                       \
    if (!(expression)) {                                                                     \
      fprintf(stderr, "android gate test failed: %s (%s:%d)\n", #expression, __FILE__,       \
              __LINE__);                                                                     \
      abort();                                                                               \
    }                                                                                        \
  } while (0)

static _Atomic(bool) workers_running = true;
static sem_t enable_request;

static void test_fork_handlers(void);

static void assert_gate_quiescent(void) {
  TEST_CHECK(atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) ==
             MIMALLOC_GATE_CLOSED);
  for (size_t i = 0; i < MIMALLOC_GATE_LANE_COUNT; i++) {
    TEST_CHECK(atomic_load_explicit(&g_mimalloc_gate_lanes[i].active, memory_order_seq_cst) == 0);
  }
}

static void* allocation_worker(void* arg) {
  uintptr_t seed = (uintptr_t)arg + 1;
  while (atomic_load_explicit(&workers_running, memory_order_relaxed)) {
    size_t size = ((seed++ * 17) & 1023) + 1;
    void* p = mimalloc_malloc(size);
    TEST_CHECK(p != NULL);
    mimalloc_free(p);
  }
  return NULL;
}

static void* cross_thread_enabler(void* arg) {
  int err;
  do {
    err = sem_wait(&enable_request);
  } while (err != 0 && errno == EINTR);
  TEST_CHECK(err == 0);

  // free(NULL) is required to be a no-op even while allocations are disabled.
  // libmemunreachable relies on this when stdio cleanup runs in its ptracer
  // thread before the original thread can re-enable the allocator.
  TEST_CHECK(arg == NULL);
  mimalloc_free(arg);
  mimalloc_malloc_enable();
  return NULL;
}

static void test_disable_under_load(void) {
  pthread_t workers[WORKER_COUNT];
  for (uintptr_t i = 0; i < WORKER_COUNT; i++) {
    TEST_CHECK(pthread_create(&workers[i], NULL, allocation_worker, (void*)i) == 0);
  }

  for (size_t i = 0; i < DISABLE_ITERATIONS; i++) {
    mimalloc_malloc_disable();
    assert_gate_quiescent();
    mimalloc_malloc_enable();
  }

  test_fork_handlers();

  atomic_store_explicit(&workers_running, false, memory_order_relaxed);
  for (size_t i = 0; i < WORKER_COUNT; i++) {
    TEST_CHECK(pthread_join(workers[i], NULL) == 0);
  }
}

static void test_cross_thread_enable(void) {
  pthread_t enabler;
  TEST_CHECK(sem_init(&enable_request, 0, 0) == 0);
  TEST_CHECK(pthread_create(&enabler, NULL, cross_thread_enabler, NULL) == 0);

  mimalloc_malloc_disable();
  assert_gate_quiescent();
  TEST_CHECK(sem_post(&enable_request) == 0);
  TEST_CHECK(pthread_join(enabler, NULL) == 0);
  TEST_CHECK(atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) ==
             MIMALLOC_GATE_OPEN);

  TEST_CHECK(sem_destroy(&enable_request) == 0);
}

static void test_fork_handlers(void) {
  pid_t pid = fork();
  TEST_CHECK(pid >= 0);
  if (pid == 0) {
    void* p = mimalloc_malloc(128);
    TEST_CHECK(p != NULL);
    mimalloc_free(p);
    mimalloc_malloc_disable();
    assert_gate_quiescent();
    mimalloc_malloc_enable();
    _exit(0);
  }

  int status;
  TEST_CHECK(waitpid(pid, &status, 0) == pid);
  TEST_CHECK(WIFEXITED(status));
  TEST_CHECK(WEXITSTATUS(status) == 0);
}

int main(void) {
  // Exercise the less common path where disable is the first gate operation.
  mimalloc_malloc_disable();
  assert_gate_quiescent();
  mimalloc_malloc_enable();

  // Exercise the normal allocation path after initialization.
  void* p = mimalloc_malloc(1);
  TEST_CHECK(p != NULL);
  mimalloc_free(p);

  test_disable_under_load();
  test_cross_thread_enable();
  return 0;
}
