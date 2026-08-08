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

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MALLOC_INFO_ALLOCATION_COUNT 64
#define MALLOC_INFO_STRESS_THREADS 4
#define MALLOC_INFO_STRESS_SLOTS 16
#define MALLOC_INFO_STRESS_SNAPSHOTS 50

#define TEST_CHECK(expression)                                                        \
  do {                                                                                \
    if (!(expression)) {                                                              \
      fprintf(stderr, "android malloc_info test failed: %s (%s:%d)\n", #expression, \
              __FILE__, __LINE__);                                                    \
      abort();                                                                        \
    }                                                                                 \
  } while (0)

typedef int (*malloc_info_fn_t)(int options, FILE* fp);
typedef void* (*malloc_fn_t)(size_t size);
typedef void (*free_fn_t)(void* pointer);

typedef struct malloc_info_summary_s {
  size_t allocated;
  size_t reusable;
  size_t capacity;
} malloc_info_summary_t;

typedef struct malloc_info_stress_state_s {
  _Atomic(size_t) ready;
  _Atomic(bool) stop;
} malloc_info_stress_state_t;

static malloc_info_summary_t capture_summary(malloc_info_fn_t dump) {
  FILE* fp = tmpfile();
  TEST_CHECK(fp != NULL);
  TEST_CHECK(dump(0, fp) == 0);
  TEST_CHECK(fseek(fp, 0, SEEK_SET) == 0);

  char xml[512];
  size_t length = fread(xml, 1, sizeof(xml) - 1, fp);
  TEST_CHECK(!ferror(fp));
  TEST_CHECK(feof(fp));
  xml[length] = '\0';
  TEST_CHECK(fclose(fp) == 0);

  TEST_CHECK(strstr(xml, "<malloc version=\"mimalloc-1\">") != NULL);
  TEST_CHECK(strstr(xml, "debug-malloc-1") == NULL);
  TEST_CHECK(strstr(xml, "</malloc>") != NULL);

  const char* summary = strstr(xml, "<summary ");
  TEST_CHECK(summary != NULL);
  malloc_info_summary_t result = {};
  TEST_CHECK(sscanf(summary,
                    "<summary allocated=\"%zu\" reusable=\"%zu\" capacity=\"%zu\"/>",
                    &result.allocated, &result.reusable, &result.capacity) == 3);
  TEST_CHECK(result.capacity >= result.allocated);
  TEST_CHECK(result.capacity - result.allocated == result.reusable);
  return result;
}

static void test_invalid_arguments(void) {
  FILE* fp = tmpfile();
  TEST_CHECK(fp != NULL);
  errno = 0;
  TEST_CHECK(mimalloc_malloc_info(1, fp) == -1);
  TEST_CHECK(errno == EINVAL);
  TEST_CHECK(fclose(fp) == 0);

  errno = 0;
  TEST_CHECK(mimalloc_malloc_info(0, NULL) == -1);
  TEST_CHECK(errno == EINVAL);
}

static void test_allocation_accounting(const char* name, malloc_info_fn_t dump,
                                       malloc_fn_t allocate, free_fn_t deallocate) {
  void* pointers[MALLOC_INFO_ALLOCATION_COUNT] = {};
  size_t requested = 0;
  malloc_info_summary_t before = capture_summary(dump);

  for (size_t i = 0; i < MALLOC_INFO_ALLOCATION_COUNT; i++) {
    size_t size = 4096 + (i + 1) * 8191;
    pointers[i] = allocate(size);
    TEST_CHECK(pointers[i] != NULL);
    memset(pointers[i], (int)i, size);
    requested += size;
  }

  malloc_info_summary_t allocated = capture_summary(dump);
  for (size_t i = 0; i < MALLOC_INFO_ALLOCATION_COUNT; i++) {
    deallocate(pointers[i]);
  }
  malloc_info_summary_t freed = capture_summary(dump);

  printf("%s before=%zu allocated=%zu freed=%zu requested=%zu\n", name,
         before.allocated, allocated.allocated, freed.allocated, requested);
  TEST_CHECK(allocated.allocated >= before.allocated + requested);
  TEST_CHECK(freed.allocated + requested <= allocated.allocated);
}

static uint32_t next_random(uint32_t* state) {
  uint32_t value = *state;
  value ^= value << 13;
  value ^= value >> 17;
  value ^= value << 5;
  *state = value;
  return value;
}

static void* stress_worker(void* argument) {
  malloc_info_stress_state_t* state = (malloc_info_stress_state_t*)argument;
  void* slots[MALLOC_INFO_STRESS_SLOTS] = {};
  uint32_t random = (uint32_t)(uintptr_t)pthread_self() ^ 0x9e3779b9U;

  for (size_t i = 0; i < MALLOC_INFO_STRESS_SLOTS; i++) {
    slots[i] = mimalloc_malloc(1 + next_random(&random) % 65536);
    TEST_CHECK(slots[i] != NULL);
  }
  atomic_fetch_add_explicit(&state->ready, 1, memory_order_release);
  while (!atomic_load_explicit(&state->stop, memory_order_acquire)) {
    size_t slot = next_random(&random) % MALLOC_INFO_STRESS_SLOTS;
    mimalloc_free(slots[slot]);
    slots[slot] = mimalloc_malloc(1 + next_random(&random) % 65536);
    TEST_CHECK(slots[slot] != NULL);
  }
  for (size_t i = 0; i < MALLOC_INFO_STRESS_SLOTS; i++) {
    mimalloc_free(slots[i]);
  }
  return NULL;
}

static void test_concurrent_snapshots(void) {
  malloc_info_stress_state_t state = {};
  pthread_t threads[MALLOC_INFO_STRESS_THREADS];
  for (size_t i = 0; i < MALLOC_INFO_STRESS_THREADS; i++) {
    TEST_CHECK(pthread_create(&threads[i], NULL, stress_worker, &state) == 0);
  }
  while (atomic_load_explicit(&state.ready, memory_order_acquire) !=
         MALLOC_INFO_STRESS_THREADS) {
    sched_yield();
  }
  for (size_t i = 0; i < MALLOC_INFO_STRESS_SNAPSHOTS; i++) {
    (void)capture_summary(mimalloc_malloc_info);
  }
  atomic_store_explicit(&state.stop, true, memory_order_release);
  for (size_t i = 0; i < MALLOC_INFO_STRESS_THREADS; i++) {
    TEST_CHECK(pthread_join(threads[i], NULL) == 0);
  }
}

static void test_reentrant_snapshot(void) {
  mimalloc_operation_begin();
  (void)capture_summary(mimalloc_malloc_info);
  mimalloc_operation_end();
}

int main(void) {
  test_invalid_arguments();
  test_allocation_accounting("direct", mimalloc_malloc_info, mimalloc_malloc, mimalloc_free);
  test_concurrent_snapshots();
  test_reentrant_snapshot();
  test_allocation_accounting("public", malloc_info, malloc, free);
  return 0;
}
