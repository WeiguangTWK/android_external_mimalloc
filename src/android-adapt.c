#include "mimalloc.h"
#include "mimalloc-stats.h"
#include "mimalloc_android_gate.h"
#include "mimalloc/types.h"

#include <limits.h>
#include <malloc.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

_Atomic(uint32_t) g_mimalloc_gate_state = MIMALLOC_GATE_UNINITIALIZED;
_Atomic(uint32_t) g_mimalloc_gate_next_lane = 0;
mimalloc_gate_lane_t g_mimalloc_gate_lanes[MIMALLOC_GATE_LANE_COUNT];
_Thread_local uint32_t g_mimalloc_gate_lane = MIMALLOC_GATE_UNASSIGNED_LANE;
_Thread_local size_t g_mimalloc_gate_reentry_depth = 0;

static pthread_mutex_t g_mimalloc_gate_wait_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_mimalloc_gate_wait_cond = PTHREAD_COND_INITIALIZER;

static bool mimalloc_gate_has_active_calls(void) {
  for (size_t i = 0; i < MIMALLOC_GATE_LANE_COUNT; i++) {
    if (atomic_load_explicit(&g_mimalloc_gate_lanes[i].active, memory_order_seq_cst) != 0) {
      return true;
    }
  }
  return false;
}

void mimalloc_gate_wait_until_open(void) {
  pthread_mutex_lock(&g_mimalloc_gate_wait_lock);
  while (atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) !=
         MIMALLOC_GATE_OPEN) {
    pthread_cond_wait(&g_mimalloc_gate_wait_cond, &g_mimalloc_gate_wait_lock);
  }
  pthread_mutex_unlock(&g_mimalloc_gate_wait_lock);
}

void mimalloc_gate_notify_lane_drained(void) {
  pthread_mutex_lock(&g_mimalloc_gate_wait_lock);
  pthread_cond_broadcast(&g_mimalloc_gate_wait_cond);
  pthread_mutex_unlock(&g_mimalloc_gate_wait_lock);
}

void mimalloc_gate_disable(void) {
  mimalloc_gate_ensure_initialized();

  for (;;) {
    uint32_t expected = MIMALLOC_GATE_OPEN;
    if (atomic_compare_exchange_strong_explicit(&g_mimalloc_gate_state, &expected,
                                                MIMALLOC_GATE_CLOSING, memory_order_seq_cst,
                                                memory_order_seq_cst)) {
      break;
    }
    mimalloc_gate_wait_until_open();
  }

  pthread_mutex_lock(&g_mimalloc_gate_wait_lock);
  while (mimalloc_gate_has_active_calls()) {
    pthread_cond_wait(&g_mimalloc_gate_wait_cond, &g_mimalloc_gate_wait_lock);
  }
  atomic_store_explicit(&g_mimalloc_gate_state, MIMALLOC_GATE_CLOSED, memory_order_seq_cst);
  pthread_cond_broadcast(&g_mimalloc_gate_wait_cond);
  pthread_mutex_unlock(&g_mimalloc_gate_wait_lock);
}

void mimalloc_gate_enable(void) {
  mimalloc_gate_ensure_initialized();

  pthread_mutex_lock(&g_mimalloc_gate_wait_lock);
  while (atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) ==
         MIMALLOC_GATE_CLOSING) {
    pthread_cond_wait(&g_mimalloc_gate_wait_cond, &g_mimalloc_gate_wait_lock);
  }
  if (atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) ==
      MIMALLOC_GATE_CLOSED) {
    atomic_store_explicit(&g_mimalloc_gate_state, MIMALLOC_GATE_OPEN, memory_order_seq_cst);
    pthread_cond_broadcast(&g_mimalloc_gate_wait_cond);
  }
  pthread_mutex_unlock(&g_mimalloc_gate_wait_lock);
}

static void mimalloc_gate_atfork_prepare(void) {
  mimalloc_gate_disable();
}

static void mimalloc_gate_atfork_parent(void) {
  mimalloc_gate_enable();
}

static void mimalloc_gate_atfork_child(void) {
  // Only the thread that called fork survives in the child. Recreate the
  // process-private slow-path synchronization objects instead of inheriting
  // waiter bookkeeping from threads that no longer exist.
  pthread_mutex_init(&g_mimalloc_gate_wait_lock, NULL);
  pthread_cond_init(&g_mimalloc_gate_wait_cond, NULL);
  for (size_t i = 0; i < MIMALLOC_GATE_LANE_COUNT; i++) {
    atomic_store_explicit(&g_mimalloc_gate_lanes[i].active, 0, memory_order_seq_cst);
  }
  g_mimalloc_gate_reentry_depth = 0;
  atomic_store_explicit(&g_mimalloc_gate_state, MIMALLOC_GATE_OPEN, memory_order_seq_cst);
}

void mimalloc_gate_ensure_initialized(void) {
  for (;;) {
    uint32_t state = atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst);
    if (state >= MIMALLOC_GATE_OPEN) return;

    if (state == MIMALLOC_GATE_UNINITIALIZED) {
      uint32_t expected = MIMALLOC_GATE_UNINITIALIZED;
      if (atomic_compare_exchange_strong_explicit(&g_mimalloc_gate_state, &expected,
                                                  MIMALLOC_GATE_INITIALIZING,
                                                  memory_order_seq_cst,
                                                  memory_order_seq_cst)) {
        bool set_reentry_guard = (g_mimalloc_gate_reentry_depth == 0);
        if (set_reentry_guard) g_mimalloc_gate_reentry_depth = 1;
        int err = pthread_atfork(mimalloc_gate_atfork_prepare, mimalloc_gate_atfork_parent,
                                 mimalloc_gate_atfork_child);
        if (set_reentry_guard) g_mimalloc_gate_reentry_depth = 0;
        // Failing to install atfork handlers must not make the allocator
        // unavailable. The gate remains usable, but fork synchronization is
        // not provided for this process.
        (void)err;

        pthread_mutex_lock(&g_mimalloc_gate_wait_lock);
        atomic_store_explicit(&g_mimalloc_gate_state, MIMALLOC_GATE_OPEN,
                              memory_order_seq_cst);
        pthread_cond_broadcast(&g_mimalloc_gate_wait_cond);
        pthread_mutex_unlock(&g_mimalloc_gate_wait_lock);
        return;
      }
      continue;
    }

    pthread_mutex_lock(&g_mimalloc_gate_wait_lock);
    while (atomic_load_explicit(&g_mimalloc_gate_state, memory_order_seq_cst) ==
           MIMALLOC_GATE_INITIALIZING) {
      pthread_cond_wait(&g_mimalloc_gate_wait_cond, &g_mimalloc_gate_wait_lock);
    }
    pthread_mutex_unlock(&g_mimalloc_gate_wait_lock);
  }
}

typedef struct mimalloc_iterate_arg_s {
  uintptr_t base;
  uintptr_t end;
  void (*callback)(uintptr_t base, size_t size, void* arg);
  void* arg;
} mimalloc_iterate_arg_t;

static int mimalloc_write_xml(FILE* fp, size_t current_allocated, size_t current_commit) {
  if (fprintf(fp,
              "<malloc version=\"debug-malloc-1\">"
              "<current_allocated>%zu</current_allocated>"
              "<current_commit>%zu</current_commit>"
              "</malloc>\n",
              current_allocated, current_commit) < 0) {
    return -1;
  }
  return 0;
}

static bool mimalloc_visit_block(const mi_heap_t* heap, const mi_heap_area_t* area, void* block,
                                 size_t block_size, void* arg) {
  (void)heap;
  (void)area;
  (void)block_size;

  if (block == NULL) return true;

  mimalloc_iterate_arg_t* iterate_arg = (mimalloc_iterate_arg_t*)arg;
  uintptr_t block_addr = (uintptr_t)block;
  if (block_addr < iterate_arg->base || block_addr >= iterate_arg->end) return true;

  size_t usable = mi_malloc_usable_size(block);
  if (usable == 0) return true;

  iterate_arg->callback(block_addr, usable, iterate_arg->arg);
  return true;
}

static bool mimalloc_segment_is_valid(const mi_segment_t* segment) {
  if (_mi_ptr_cookie(segment) != segment->cookie) return false;
  if (segment->segment_size == 0 ||
      segment->segment_size % MI_SEGMENT_SLICE_SIZE != 0) {
    return false;
  }
  if (segment->kind != MI_SEGMENT_NORMAL && segment->kind != MI_SEGMENT_HUGE) return false;
  if (segment->slice_entries == 0 || segment->slice_entries > MI_SLICES_PER_SEGMENT) return false;
  if (segment->segment_info_slices == 0 ||
      segment->segment_info_slices > segment->slice_entries) {
    return false;
  }
  return true;
}

static void mimalloc_visit_segment(mi_segment_t* segment, mimalloc_iterate_arg_t* iterate_arg) {
  if (segment->used == 0) return;
  (void)_mi_segment_visit_blocks(segment, -1, true, mimalloc_visit_block, iterate_arg);
}

static void mimalloc_visit_arena_segments(mimalloc_iterate_arg_t* iterate_arg) {
  size_t arena_count = mi_arena_get_count();
  if (arena_count > MI_MAX_ARENAS) arena_count = MI_MAX_ARENAS;

  for (size_t arena_index = 0; arena_index < arena_count; arena_index++) {
    mi_arena_t* arena = mi_arena_from_index(arena_index);
    if (arena == NULL) continue;

    for (size_t field = 0; field < arena->field_count; field++) {
      size_t inuse = mi_atomic_load_relaxed(&arena->blocks_inuse[field]);
      while (inuse != 0) {
        const size_t bit = mi_ctz(inuse);
        const mi_bitmap_index_t block_index = mi_bitmap_index_create(field, bit);
        inuse &= inuse - 1;
        if (mi_bitmap_index_bit(block_index) >= arena->block_count) continue;

        mi_segment_t* segment = (mi_segment_t*)mi_arena_block_start(arena, block_index);
        if (segment->memid.memkind != MI_MEM_ARENA ||
            segment->memid.mem.arena.id != arena->id ||
            segment->memid.mem.arena.block_index != block_index ||
            !mimalloc_segment_is_valid(segment)) {
          continue;
        }
        mimalloc_visit_segment(segment, iterate_arg);
      }
    }
  }
}

static void mimalloc_visit_os_segments(mimalloc_iterate_arg_t* iterate_arg) {
  for (size_t part_index = 0; part_index < MI_SEGMENT_MAP_MAX_PARTS; part_index++) {
    mi_segmap_part_t* part =
        mi_atomic_load_ptr_acquire(mi_segmap_part_t, &mi_segment_map[part_index]);
    if (part == NULL) continue;

    for (size_t field = 0; field < MI_SEGMENT_MAP_PART_ENTRIES; field++) {
      uintptr_t allocated = mi_atomic_load_relaxed(&part->map[field]);
      while (allocated != 0) {
        const size_t bit = mi_ctz(allocated);
        allocated &= allocated - 1;

        const uintptr_t part_base = (uintptr_t)part_index * MI_SEGMENT_MAP_PART_SPAN;
        const uintptr_t bit_index = (uintptr_t)field * MI_INTPTR_BITS + bit;
        mi_segment_t* segment =
            (mi_segment_t*)(part_base + bit_index * MI_SEGMENT_MAP_PART_BIT_SPAN);
        if (segment->memid.memkind == MI_MEM_ARENA ||
            !mimalloc_segment_is_valid(segment)) {
          continue;
        }
        mimalloc_visit_segment(segment, iterate_arg);
      }
    }
  }
}

mi_decl_export struct mallinfo mimalloc_helper_mallinfo(void) {
  struct mallinfo info = {};
  mi_stats_t_decl(stats);
  if (mi_stats_get(&stats)) {
    size_t current_allocated = (size_t)(stats.malloc_normal.current + stats.malloc_huge.current);
    size_t current_commit = (size_t)stats.committed.current;
    info.uordblks = current_allocated > INT_MAX ? INT_MAX : (int)current_allocated;
    info.hblkhd = current_commit > INT_MAX ? INT_MAX : (int)current_commit;
    if (current_commit >= current_allocated) {
      size_t free_bytes = current_commit - current_allocated;
      info.fordblks = free_bytes > INT_MAX ? INT_MAX : (int)free_bytes;
    }
  }
  return info;
}

mi_decl_export int mimalloc_helper_malloc_info(int options, FILE* fp) {
  if (options != 0) {
    return -1;
  }
  if (fp == NULL) {
    return -1;
  }

  mi_stats_t_decl(stats);
  size_t current_allocated = 0;
  size_t current_commit = 0;
  if (mi_stats_get(&stats)) {
    current_allocated = (size_t)(stats.malloc_normal.current + stats.malloc_huge.current);
    current_commit = (size_t)stats.committed.current;
  }

  fflush(fp);
  if (mimalloc_write_xml(fp, current_allocated, current_commit) != 0) {
    return -1;
  }
  return fflush(fp);
}

mi_decl_export int mimalloc_helper_malloc_iterate(
    uintptr_t base, size_t size, void (*callback)(uintptr_t base, size_t size, void* arg),
    void* arg) {
  uintptr_t end;
  if (__builtin_add_overflow(base, size, &end)) {
    return 0;
  }

  mimalloc_iterate_arg_t iterate_arg;
  iterate_arg.base = base;
  iterate_arg.end = end;
  iterate_arg.callback = callback;
  iterate_arg.arg = arg;

  mimalloc_visit_arena_segments(&iterate_arg);
  mimalloc_visit_os_segments(&iterate_arg);
  return 0;
}
