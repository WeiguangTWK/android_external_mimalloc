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

#include <stdio.h>
#include <stdlib.h>

#define TEST_CHECK(expression)                                                            \
  do {                                                                                    \
    if (!(expression)) {                                                                  \
      fprintf(stderr, "android mallopt test failed: %s (%s:%d)\n", #expression, __FILE__, \
              __LINE__);                                                                  \
      abort();                                                                            \
    }                                                                                     \
  } while (0)

typedef struct mallopt_case_s {
  int option;
  int value;
} mallopt_case_t;

static const mallopt_case_t unsupported_allocator_options[] = {
    {M_MEMTAG_TUNING, M_MEMTAG_TUNING_BUFFER_OVERFLOW},
    {M_MEMTAG_TUNING, M_MEMTAG_TUNING_UAF},
    {M_THREAD_DISABLE_MEM_INIT, 0},
    {M_THREAD_DISABLE_MEM_INIT, 1},
    {M_CACHE_COUNT_MAX, 100},
    {M_CACHE_SIZE_MAX, 2 * 1024 * 1024},
    {M_TSDS_COUNT_MAX, 8},
};

static void test_direct_adapter_contract(void) {
  for (size_t i = 0;
       i < sizeof(unsupported_allocator_options) / sizeof(unsupported_allocator_options[0]);
       i++) {
    TEST_CHECK(mimalloc_mallopt(unsupported_allocator_options[i].option,
                                unsupported_allocator_options[i].value) == 0);
  }

  // These options belong to bionic and must not be claimed by the allocator.
  TEST_CHECK(mimalloc_mallopt(M_BIONIC_ZERO_INIT, 1) == 0);
  TEST_CHECK(mimalloc_mallopt(M_BIONIC_SET_HEAP_TAGGING_LEVEL,
                              M_HEAP_TAGGING_LEVEL_NONE) == 0);

  TEST_CHECK(mimalloc_mallopt(-1000, 1) == 0);
  TEST_CHECK(mimalloc_mallopt(M_DECAY_TIME, -2) == 0);
  TEST_CHECK(mimalloc_mallopt(M_DECAY_TIME, 2) == 0);
}

static void test_public_allocator_contract(void) {
  for (size_t i = 0;
       i < sizeof(unsupported_allocator_options) / sizeof(unsupported_allocator_options[0]);
       i++) {
    TEST_CHECK(mallopt(unsupported_allocator_options[i].option,
                       unsupported_allocator_options[i].value) == 0);
  }

  TEST_CHECK(mallopt(-1000, 1) == 0);
}

int main(void) {
  test_direct_adapter_contract();
  test_public_allocator_contract();
  return 0;
}
