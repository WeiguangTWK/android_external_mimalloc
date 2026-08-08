#include "mimalloc.h"
#include "mimalloc_android_gate.h"
#include "mimalloc/types.h"

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
  // Do not clear lane->allocated: every live parent allocation is inherited
  // by the child even though only the calling thread survives fork.
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

static int mimalloc_write_xml(FILE* fp, struct mallinfo info) {
  if (fprintf(fp,
              "<malloc version=\"mimalloc-1\">\n"
              "  <summary allocated=\"%zu\" reusable=\"%zu\" capacity=\"%zu\"/>\n"
              "</malloc>\n",
              info.uordblks, info.fordblks, info.usmblks) < 0) {
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

typedef void(mimalloc_segment_visitor_t)(mi_segment_t* segment, void* arg);

static void mimalloc_visit_arena_segments(mimalloc_segment_visitor_t* visitor, void* arg) {
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
        if (segment->used != 0) visitor(segment, arg);
      }
    }
  }
}

static void mimalloc_visit_os_segments(mimalloc_segment_visitor_t* visitor, void* arg) {
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
        if (segment->used != 0) visitor(segment, arg);
      }
    }
  }
}

static void mimalloc_visit_all_segments(mimalloc_segment_visitor_t* visitor, void* arg) {
  mimalloc_visit_arena_segments(visitor, arg);
  mimalloc_visit_os_segments(visitor, arg);
}

static void mimalloc_iterate_segment(mi_segment_t* segment, void* arg) {
  (void)_mi_segment_visit_blocks(segment, -1, true, mimalloc_visit_block, arg);
}

typedef struct mimalloc_find_heap_arg_s {
  uintptr_t after;
  mi_heap_t* next;
} mimalloc_find_heap_arg_t;

static void mimalloc_find_next_heap_in_segment(mi_segment_t* segment, void* arg) {
  mimalloc_find_heap_arg_t* find = (mimalloc_find_heap_arg_t*)arg;
  const mi_slice_t* end;
  mi_slice_t* slice = mi_slices_start_iterate(segment, &end);

  while (slice < end) {
    if (mi_slice_is_used(slice)) {
      mi_heap_t* heap = mi_page_heap(mi_slice_to_page(slice));
      const uintptr_t address = (uintptr_t)heap;
      if (address > find->after &&
          (find->next == NULL || address < (uintptr_t)find->next)) {
        find->next = heap;
      }
    }
    slice += slice->slice_count;
  }
}

typedef struct mimalloc_find_purge_segment_arg_s {
  uintptr_t after;
  mi_segment_t* next;
} mimalloc_find_purge_segment_arg_t;

// Access is serialized by the closed process-wide gate. Keep only the numeric
// address so a segment disappearing between purge calls cannot leave a stale
// pointer that is ever dereferenced.
static uintptr_t g_mimalloc_purge_segment_cursor;

static void mimalloc_select_next_purge_segment(mi_segment_t* segment, void* arg) {
  if (!segment->allow_purge || segment->purge_expire == 0 ||
      mi_commit_mask_is_empty(&segment->purge_mask)) {
    return;
  }

  mimalloc_find_purge_segment_arg_t* find =
      (mimalloc_find_purge_segment_arg_t*)arg;
  const uintptr_t address = (uintptr_t)segment;
  if (address > find->after &&
      (find->next == NULL || address < (uintptr_t)find->next)) {
    find->next = segment;
  }
}

static mi_segment_t* mimalloc_find_next_purge_segment(uintptr_t after) {
  mimalloc_find_purge_segment_arg_t find = {.after = after, .next = NULL};
  mimalloc_visit_all_segments(mimalloc_select_next_purge_segment, &find);
  return find.next;
}

mi_decl_export void mimalloc_helper_purge(void) {
  // Release everything immediately available from the caller's heap first.
  // This also performs the process-wide arena purge once.
  mi_collect(true);

  // Process at most one other live segment per normal purge. Repeated Android
  // idle or resource purges advance the cursor, while M_PURGE_ALL remains the
  // exhaustive operation for delayed frees and every live heap. The public
  // wrapper holds the gate, so the segment indexes and cursor are stable here.
  mi_segment_t* segment =
      mimalloc_find_next_purge_segment(g_mimalloc_purge_segment_cursor);
  if (segment == NULL && g_mimalloc_purge_segment_cursor != 0) {
    segment = mimalloc_find_next_purge_segment(0);
  }

  if (segment == NULL) {
    g_mimalloc_purge_segment_cursor = 0;
  } else {
    g_mimalloc_purge_segment_cursor = (uintptr_t)segment;
    _mi_segment_collect(segment, true);
  }
}

mi_decl_export void mimalloc_helper_purge_all(void) {
  // Collect the caller's default heap first. Besides being inexpensive, this
  // also handles abandoned segments in the default subprocess.
  mi_collect(true);

  // Collecting a heap may free its segments and mutate the global indexes.
  // Find one heap at a time in address order and restart the scan after each
  // collection. This avoids allocating a temporary registry while the gate is
  // closed and keeps all bookkeeping off the allocation hot path.
  uintptr_t after = 0;
  for (;;) {
    mimalloc_find_heap_arg_t find = {.after = after, .next = NULL};
    mimalloc_visit_all_segments(mimalloc_find_next_heap_in_segment, &find);
    if (find.next == NULL) break;
    mi_heap_collect(find.next, true);
    after = (uintptr_t)find.next;
  }

  // Heap collection already visits arenas, but do one final forced pass after
  // all pages and segments released above have been scheduled for purging.
  _mi_arenas_collect(true);
}

typedef struct mimalloc_mallinfo_snapshot_s {
  size_t allocated;
  size_t free;
} mimalloc_mallinfo_snapshot_t;

static size_t mimalloc_saturating_multiply(size_t left, size_t right) {
  if (left != 0 && right > SIZE_MAX / left) return SIZE_MAX;
  return left * right;
}

static void mimalloc_saturating_add(size_t* total, size_t value) {
  if (value > SIZE_MAX - *total) {
    *total = SIZE_MAX;
  } else {
    *total += value;
  }
}

static void mimalloc_saturating_subtract(size_t* total, size_t value) {
  *total = (value >= *total ? 0 : *total - value);
}

static void mimalloc_snapshot_segment(mi_segment_t* segment, void* arg) {
  mimalloc_mallinfo_snapshot_t* snapshot = (mimalloc_mallinfo_snapshot_t*)arg;
  const mi_slice_t* end;
  mi_slice_t* slice = mi_slices_start_iterate(segment, &end);

  while (slice < end) {
    if (mi_slice_is_used(slice)) {
      mi_page_t* page = mi_slice_to_page(slice);
      _mi_page_free_collect(page, true);
      const size_t usable = mi_page_usable_block_size(page);
      const size_t used = page->used;
      const size_t available = (used <= page->capacity ? page->capacity - used : 0);
      mimalloc_saturating_add(
          &snapshot->allocated, mimalloc_saturating_multiply(used, usable));
      mimalloc_saturating_add(
          &snapshot->free, mimalloc_saturating_multiply(available, usable));
    }
    slice += slice->slice_count;
  }
}

static void mimalloc_snapshot_heap_delayed_frees(mi_heap_t* heap,
                                                 mimalloc_mallinfo_snapshot_t* snapshot) {
  // The first cross-thread free from a full page is linked on the owning
  // heap instead of page->xthread_free. It is already logically free, but is
  // still included in page->used until the owner processes the delayed list.
  // The gate makes this list stable, so account for it without processing it
  // from a thread that does not own the heap.
  mi_block_t* block =
      mi_atomic_load_ptr_acquire(mi_block_t, &heap->thread_delayed_free);
  while (block != NULL) {
    mi_segment_t* segment = _mi_ptr_segment(block);
    mi_page_t* page = _mi_segment_page_of(segment, block);
    const size_t usable = mi_page_usable_block_size(page);
    mimalloc_saturating_subtract(&snapshot->allocated, usable);
    mimalloc_saturating_add(&snapshot->free, usable);
    block = mi_block_nextx(heap, block, heap->keys);
  }
}

static void mimalloc_snapshot_all_delayed_frees(
    mimalloc_mallinfo_snapshot_t* snapshot) {
  uintptr_t after = 0;
  for (;;) {
    mimalloc_find_heap_arg_t find = {.after = after, .next = NULL};
    mimalloc_visit_all_segments(mimalloc_find_next_heap_in_segment, &find);
    if (find.next == NULL) break;
    mimalloc_snapshot_heap_delayed_frees(find.next, snapshot);
    after = (uintptr_t)find.next;
  }
}

mi_decl_export struct mallinfo mimalloc_helper_mallinfo_snapshot(void) {
  struct mallinfo info = {};
  mimalloc_mallinfo_snapshot_t snapshot = {};
  mimalloc_visit_all_segments(mimalloc_snapshot_segment, &snapshot);
  mimalloc_snapshot_all_delayed_frees(&snapshot);

  info.uordblks = snapshot.allocated;
  // This is a conservative lower bound: immediately reusable capacity in
  // active pages. It deliberately excludes metadata and unassigned spans.
  info.fsmblks = snapshot.free;
  info.fordblks = snapshot.free;
  // Android exposes usmblks as Native Heap Size. Arena reservations and even
  // assigned segments can be orders of magnitude larger than their live
  // pages. Report the usable capacity managed by active pages so Heap Size is
  // the saturating sum of Heap Alloc and Heap Free.
  info.hblkhd = info.uordblks;
  mimalloc_saturating_add(&info.hblkhd, info.fordblks);
  info.usmblks = info.hblkhd;
  return info;
}

mi_decl_export struct mallinfo mimalloc_helper_mallinfo(void) {
  struct mallinfo info = {};
  uint64_t allocated = 0;
  uint64_t freed = 0;
  for (size_t i = 0; i < MIMALLOC_GATE_LANE_COUNT; i++) {
    const int64_t lane = atomic_load_explicit(&g_mimalloc_gate_lanes[i].allocated,
                                              memory_order_relaxed);
    if (lane >= 0) {
      const uint64_t value = (uint64_t)lane;
      allocated = (value > UINT64_MAX - allocated ? UINT64_MAX : allocated + value);
    } else {
      // Avoid negating INT64_MIN. A lane can be negative when this thread
      // frees memory allocated by a thread assigned to a different lane.
      const uint64_t value = (uint64_t)(-(lane + 1)) + 1;
      freed = (value > UINT64_MAX - freed ? UINT64_MAX : freed + value);
    }
  }
  const uint64_t live = (allocated > freed ? allocated - freed : 0);
#if SIZE_MAX < UINT64_MAX
  if (live > SIZE_MAX) {
    info.uordblks = SIZE_MAX;
  } else {
    info.uordblks = (size_t)live;
  }
#else
  info.uordblks = (size_t)live;
#endif
  // Without walking every page we do not know how much committed memory is
  // immediately reusable. Android's hot users need live bytes, so report no
  // reusable estimate and make capacity equal to the known live allocation.
  info.hblkhd = info.uordblks;
  info.usmblks = info.uordblks;
  return info;
}

mi_decl_export struct mallinfo mimalloc_helper_mallinfo_reentrant(void) {
  return mimalloc_helper_mallinfo();
}

mi_decl_export int mimalloc_helper_malloc_info(FILE* fp, struct mallinfo info) {
  if (mimalloc_write_xml(fp, info) != 0) {
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

  mimalloc_visit_all_segments(mimalloc_iterate_segment, &iterate_arg);
  return 0;
}
