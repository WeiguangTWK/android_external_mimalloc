/*
 * Copyright (C) 2024 Neko LineageOS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#pragma once

#include <errno.h>
#include <stdbool.h>
#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <async_safe/log.h>

#include "mimalloc_android_gate.h"

__BEGIN_DECLS

void* mi_malloc(size_t size);
void* mi_calloc(size_t count, size_t size);
void* mi_realloc(void* p, size_t newsize);
void mi_free(void* p);
size_t mi_malloc_usable_size(const void* p);
void* mi_memalign(size_t alignment, size_t size);
int mi_posix_memalign(void** p, size_t alignment, size_t size);
void* mi_aligned_alloc(size_t alignment, size_t size);
void mi_collect(bool force);
#ifndef MIMALLOC_H
void mi_option_set(int option, long value);
#endif
typedef void mi_output_fun(const char* msg, void* arg);
void mi_stats_print_out(mi_output_fun* out, void* arg);
struct mallinfo mimalloc_helper_mallinfo(void);
struct mallinfo mimalloc_helper_mallinfo_reentrant(void);
int mimalloc_helper_malloc_info(FILE* fp, struct mallinfo info);
void mimalloc_helper_purge_all(void);
int mimalloc_helper_malloc_iterate(uintptr_t base, size_t size,
                                   void (*callback)(uintptr_t base, size_t size, void* arg),
                                   void* arg);
#if defined(HAVE_DEPRECATED_MALLOC_FUNCS)
void* mi_valloc(size_t size);
void* mi_pvalloc(size_t size);
#endif

#ifndef MIMALLOC_H
enum { mi_option_purge_delay = 15 };
#endif

static inline size_t mimalloc_next_pow2(size_t value) {
  if (value <= 1) return 1;
  const size_t original = value;
  value--;
  for (size_t shift = 1; shift < sizeof(value) * 8; shift <<= 1) {
    value |= value >> shift;
  }
  value++;
  return (value < original ? 0 : value);
}

static inline size_t mimalloc_memalign_alignment(size_t alignment) {
  alignment = mimalloc_next_pow2(alignment);
  if (alignment < sizeof(void*)) alignment = sizeof(void*);
  return alignment;
}

static inline void mimalloc_operation_begin() {
  mimalloc_gate_enter();
}

static inline void mimalloc_operation_end() {
  mimalloc_gate_leave();
}

static inline void mimalloc_log_output(const char* msg, void* arg) {
  (void)arg;
  async_safe_format_log(ANDROID_LOG_INFO, "mimalloc", "%s", msg);
}

static inline void* mimalloc_aligned_alloc(size_t alignment, size_t size) {
  if (alignment == 0 || (alignment & (alignment - 1)) != 0 || (size % alignment) != 0) {
    errno = EINVAL;
    return NULL;
  }
  mimalloc_operation_begin();
  void* p = mi_aligned_alloc(alignment, size);
  mimalloc_operation_end();
  return p;
}

static inline void* mimalloc_calloc(size_t n_elements, size_t elem_size) {
  mimalloc_operation_begin();
  void* p = mi_calloc(n_elements, elem_size);
  mimalloc_operation_end();
  return p;
}

static inline void mimalloc_free(void* mem) {
  if (mem == NULL) return;
  mimalloc_operation_begin();
  mi_free(mem);
  mimalloc_operation_end();
}

static inline struct mallinfo mimalloc_mallinfo() {
  if (g_mimalloc_gate_reentry_depth != 0) {
    return mimalloc_helper_mallinfo_reentrant();
  }
  mimalloc_gate_disable();
  g_mimalloc_gate_reentry_depth = 1;
  struct mallinfo info = mimalloc_helper_mallinfo();
  g_mimalloc_gate_reentry_depth = 0;
  mimalloc_gate_enable();
  return info;
}

static inline void* mimalloc_malloc(size_t bytes) {
  mimalloc_operation_begin();
  void* p = mi_malloc(bytes);
  mimalloc_operation_end();
  return p;
}

static inline int mimalloc_malloc_info(int options, FILE* fp) {
  if (options != 0) {
    errno = EINVAL;
    return -1;
  }
  if (fp == NULL) {
    errno = EINVAL;
    return -1;
  }

  // Flush pending output before taking the allocator snapshot. Stdio may
  // allocate, and file I/O must not extend the process-wide gate pause.
  if (fflush(fp) != 0) return -1;

  struct mallinfo info;
  if (g_mimalloc_gate_reentry_depth != 0) {
    info = mimalloc_helper_mallinfo_reentrant();
  } else {
    mimalloc_gate_disable();
    g_mimalloc_gate_reentry_depth = 1;
    info = mimalloc_helper_mallinfo();
    g_mimalloc_gate_reentry_depth = 0;
    mimalloc_gate_enable();
  }

  return mimalloc_helper_malloc_info(fp, info);
}

static inline size_t mimalloc_malloc_usable_size(const void* mem) {
  return mi_malloc_usable_size(mem);
}

static inline int mimalloc_mallopt(int param, int value) {
  switch (param) {
    case M_DECAY_TIME:
      if (value < -1 || value > 1) return 0;
      mimalloc_operation_begin();
      if (value < 0) {
        mi_option_set(mi_option_purge_delay, -1);
      } else if (value == 0) {
        mi_option_set(mi_option_purge_delay, 0);
      } else {
        mi_option_set(mi_option_purge_delay, 1000);
      }
      mimalloc_operation_end();
      return 1;
    case M_PURGE:
      mimalloc_operation_begin();
      mi_collect(true);
      mimalloc_operation_end();
      return 1;
    case M_PURGE_ALL:
      // Enter from outside the gate: closing it while counted as an active
      // operation would wait for this call's own lane to drain.
      if (g_mimalloc_gate_reentry_depth != 0) {
        mi_collect(true);
        return 1;
      }
      mimalloc_gate_disable();
      // Internal deferred-free callbacks may reenter the public allocation
      // wrappers. Only this purge thread is allowed through the closed gate.
      g_mimalloc_gate_reentry_depth = 1;
      mimalloc_helper_purge_all();
      g_mimalloc_gate_reentry_depth = 0;
      mimalloc_gate_enable();
      return 1;
    case M_MEMTAG_TUNING:
    case M_THREAD_DISABLE_MEM_INIT:
    case M_CACHE_COUNT_MAX:
    case M_CACHE_SIZE_MAX:
    case M_TSDS_COUNT_MAX:
    case M_BIONIC_ZERO_INIT:
    case M_BIONIC_SET_HEAP_TAGGING_LEVEL:
      (void)value;
      return 1;
    case M_LOG_STATS:
      (void)value;
      mimalloc_operation_begin();
      mi_stats_print_out(mimalloc_log_output, NULL);
      mimalloc_operation_end();
      return 1;
    default:
      return 0;
  }
}

static inline void* mimalloc_memalign(size_t alignment, size_t bytes) {
  alignment = mimalloc_memalign_alignment(alignment);
  if (alignment == 0) {
    errno = EINVAL;
    return NULL;
  }
  mimalloc_operation_begin();
  void* p = mi_memalign(alignment, bytes);
  mimalloc_operation_end();
  return p;
}

static inline void* mimalloc_realloc(void* old_mem, size_t bytes) {
  mimalloc_operation_begin();
  void* p = mi_realloc(old_mem, bytes);
  mimalloc_operation_end();
  return p;
}

static inline int mimalloc_posix_memalign(void** memptr, size_t alignment, size_t size) {
  mimalloc_operation_begin();
  int result = mi_posix_memalign(memptr, alignment, size);
  mimalloc_operation_end();
  return result;
}

#if defined(HAVE_DEPRECATED_MALLOC_FUNCS)
static inline void* mimalloc_pvalloc(size_t bytes) {
  mimalloc_operation_begin();
  void* p = mi_pvalloc(bytes);
  mimalloc_operation_end();
  return p;
}

static inline void* mimalloc_valloc(size_t bytes) {
  mimalloc_operation_begin();
  void* p = mi_valloc(bytes);
  mimalloc_operation_end();
  return p;
}
#endif

static inline int mimalloc_malloc_iterate(uintptr_t base, size_t size,
                                          void (*callback)(uintptr_t base, size_t size, void* arg),
                                          void* arg) {
  return mimalloc_helper_malloc_iterate(base, size, callback, arg);
}

static inline void mimalloc_malloc_disable() {
  mimalloc_gate_disable();
}

static inline void mimalloc_malloc_enable() {
  mimalloc_gate_enable();
}

__END_DECLS
