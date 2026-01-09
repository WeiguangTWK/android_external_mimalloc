/*
 * Copyright (C) 2024 The Android Open Source Project
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
#include <malloc.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdatomic.h>

#include <private/bionic_config.h>

#include <async_safe/log.h>

__BEGIN_DECLS

// Minimal C API declarations to avoid pulling C++ STL headers from mimalloc.h.
void* mi_malloc(size_t size);
void* mi_calloc(size_t count, size_t size);
void* mi_realloc(void* p, size_t newsize);
void  mi_free(void* p);
size_t mi_malloc_usable_size(const void* p);
void* mi_memalign(size_t alignment, size_t size);
int   mi_posix_memalign(void** p, size_t alignment, size_t size);
void* mi_aligned_alloc(size_t alignment, size_t size);
#if defined(HAVE_DEPRECATED_MALLOC_FUNCS)
void* mi_valloc(size_t size);
void* mi_pvalloc(size_t size);
#endif

static inline void* mimalloc_aligned_alloc(size_t alignment, size_t size) {
  return mi_aligned_alloc(alignment, size);
}

static inline void* mimalloc_calloc(size_t n_elements, size_t elem_size) {
  return mi_calloc(n_elements, elem_size);
}

static inline void mimalloc_free(void* mem) {
  mi_free(mem);
}

static inline struct mallinfo mimalloc_mallinfo() {
  struct mallinfo info = {};
  return info;
}

static inline void* mimalloc_malloc(size_t bytes) {
  return mi_malloc(bytes);
}

static inline int mimalloc_malloc_info(int options, FILE* fp) {
  (void)options;
  (void)fp;
  errno = ENOTSUP;
  return -1;
}

static inline size_t mimalloc_malloc_usable_size(const void* mem) {
  return mi_malloc_usable_size(mem);
}

static inline int mimalloc_mallopt(int param, int value) {
  (void)param;
  (void)value;
  return 0;
}

static inline void* mimalloc_memalign(size_t alignment, size_t bytes) {
  return mi_memalign(alignment, bytes);
}

static inline void* mimalloc_realloc(void* old_mem, size_t bytes) {
  return mi_realloc(old_mem, bytes);
}

static inline int mimalloc_posix_memalign(void** memptr, size_t alignment, size_t size) {
  return mi_posix_memalign(memptr, alignment, size);
}

#if defined(HAVE_DEPRECATED_MALLOC_FUNCS)
static inline void* mimalloc_pvalloc(size_t bytes) {
  return mi_pvalloc(bytes);
}

static inline void* mimalloc_valloc(size_t bytes) {
  return mi_valloc(bytes);
}
#endif

static inline int mimalloc_malloc_iterate(uintptr_t base, size_t size,
                                          void (*callback)(uintptr_t base, size_t size, void* arg),
                                          void* arg) {
  (void)base;
  (void)size;
  (void)callback;
  (void)arg;
  return 0;
}

static inline void mimalloc_malloc_disable() {}

static inline void mimalloc_malloc_enable() {}

__END_DECLS
