#include "mimalloc.h"
#include "mimalloc-stats.h"
#include "mimalloc/types.h"

#include <limits.h>
#include <malloc.h>
#include <stdint.h>
#include <stdio.h>

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

  mi_heap_t* backing = mi_heap_get_backing();
  if (backing != NULL && backing->tld != NULL) {
    for (mi_heap_t* heap = backing->tld->heaps; heap != NULL; heap = heap->next) {
      mi_heap_visit_blocks(heap, true, mimalloc_visit_block, &iterate_arg);
    }
    mi_abandoned_visit_blocks(backing->tld->segments.subproc, -1, true, mimalloc_visit_block,
                              &iterate_arg);
  }
  return 0;
}
