/*
 * Copyright (C) 2026 Neko LineageOS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#define MIMALLOC_GATE_THREAD_LOCAL thread_local
#else
#define MIMALLOC_GATE_THREAD_LOCAL _Thread_local
#endif

// Keep independent counters on independent cache lines. The lane count is a
// power of two so assigning a lane only needs a mask operation.
#define MIMALLOC_GATE_CACHE_LINE_SIZE 64U
#if UINTPTR_MAX > UINT32_MAX
#define MIMALLOC_GATE_LANE_COUNT 128U
#else
#define MIMALLOC_GATE_LANE_COUNT 64U
#endif
#define MIMALLOC_GATE_UNASSIGNED_LANE UINT32_MAX

typedef enum mimalloc_gate_state_e {
  MIMALLOC_GATE_UNINITIALIZED = 0,
  MIMALLOC_GATE_INITIALIZING,
  MIMALLOC_GATE_OPEN,
  MIMALLOC_GATE_CLOSING,
  MIMALLOC_GATE_CLOSED,
} mimalloc_gate_state_t;

typedef struct __attribute__((aligned(MIMALLOC_GATE_CACHE_LINE_SIZE))) mimalloc_gate_lane_s {
  _Atomic(uint32_t) active;
} mimalloc_gate_lane_t;

#ifdef __cplusplus
static_assert(sizeof(mimalloc_gate_lane_t) == MIMALLOC_GATE_CACHE_LINE_SIZE,
              "gate lanes must occupy one cache line");
static_assert((MIMALLOC_GATE_LANE_COUNT & (MIMALLOC_GATE_LANE_COUNT - 1)) == 0,
              "gate lane count must be a power of two");
#else
_Static_assert(sizeof(mimalloc_gate_lane_t) == MIMALLOC_GATE_CACHE_LINE_SIZE,
               "gate lanes must occupy one cache line");
_Static_assert((MIMALLOC_GATE_LANE_COUNT & (MIMALLOC_GATE_LANE_COUNT - 1)) == 0,
               "gate lane count must be a power of two");
#endif

extern _Atomic(uint32_t) g_mimalloc_gate_state;
extern _Atomic(uint32_t) g_mimalloc_gate_next_lane;
extern mimalloc_gate_lane_t g_mimalloc_gate_lanes[MIMALLOC_GATE_LANE_COUNT];
extern MIMALLOC_GATE_THREAD_LOCAL uint32_t g_mimalloc_gate_lane;
extern MIMALLOC_GATE_THREAD_LOCAL size_t g_mimalloc_gate_reentry_depth;

void mimalloc_gate_ensure_initialized(void);
void mimalloc_gate_wait_until_open(void);
void mimalloc_gate_notify_lane_drained(void);
void mimalloc_gate_disable(void);
void mimalloc_gate_enable(void);

static inline void mimalloc_gate_enter(void) {
  if (g_mimalloc_gate_reentry_depth != 0) {
    g_mimalloc_gate_reentry_depth++;
    return;
  }

  uint32_t state = atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst);
  if (state == MIMALLOC_GATE_UNINITIALIZED || state == MIMALLOC_GATE_INITIALIZING) {
    mimalloc_gate_ensure_initialized();
    state = atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst);
  }

  if (g_mimalloc_gate_lane == MIMALLOC_GATE_UNASSIGNED_LANE) {
    g_mimalloc_gate_lane =
        atomic_fetch_add_explicit(&g_mimalloc_gate_next_lane, 1, memory_order_relaxed) &
        (MIMALLOC_GATE_LANE_COUNT - 1);
  }

  mimalloc_gate_lane_t* lane = &g_mimalloc_gate_lanes[g_mimalloc_gate_lane];
  for (;;) {
    if (state != MIMALLOC_GATE_OPEN) {
      mimalloc_gate_wait_until_open();
      state = atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst);
      continue;
    }

    atomic_fetch_add_explicit(&lane->active, 1, memory_order_seq_cst);
    if (atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) ==
        MIMALLOC_GATE_OPEN) {
      g_mimalloc_gate_reentry_depth = 1;
      return;
    }

    // A disabler closed the gate between our first state check and the lane
    // registration. Withdraw without entering the allocator and retry after
    // the gate is opened again.
    uint32_t previous = atomic_fetch_sub_explicit(&lane->active, 1, memory_order_seq_cst);
    if (previous == 1) mimalloc_gate_notify_lane_drained();
    mimalloc_gate_wait_until_open();
    state = atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst);
  }
}

static inline void mimalloc_gate_leave(void) {
  if (--g_mimalloc_gate_reentry_depth != 0) return;

  mimalloc_gate_lane_t* lane = &g_mimalloc_gate_lanes[g_mimalloc_gate_lane];
  uint32_t previous = atomic_fetch_sub_explicit(&lane->active, 1, memory_order_seq_cst);
  if (previous == 1 &&
      atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) != MIMALLOC_GATE_OPEN) {
    mimalloc_gate_notify_lane_drained();
  }
}

#ifdef __cplusplus
}
#endif

#undef MIMALLOC_GATE_THREAD_LOCAL
