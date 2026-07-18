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

// Keep Android-specific access to mimalloc's process-wide segment indexes in
// the static library translation unit. This avoids exporting upstream-private
// arena and segment-map implementation details.
#define MI_MMAP_ANON_NAME "libc_malloc"
#include "static.c"
#include "android-adapt.c"
