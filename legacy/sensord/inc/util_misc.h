/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Copyright (C) 2023 Robert Bosch GmbH. All rights reserved.
 * Copyright (C) 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 * Copyright (C) 2008 The Android Open Source Project
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

#ifndef __UTIL_MISC_H
#define __UTIL_MISC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ABS(x) ((x) > 0 ? (x) : -(x))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) ((int)(sizeof(arr) / sizeof((arr)[0])))
#endif

#ifndef OFFSET_OF
#define OFFSET_OF(type, member) ((size_t) & ((type *)0)->member)
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) \
  (type *)((char *)(ptr)-OFFSET_OF(type, member))
#endif

#define ARRAY_ELEMENTS(array) (sizeof(array) / sizeof(array[0]))

#define FLOAT_EQU(x, y) ((x > y) ? ((x - y) < 0.000001) : ((y - x) < 0.000001))

#define EMIT_BUG()                                                     \
  do {                                                                 \
    fprintf(stderr, "bug at file: %s line: %d\n", __FILE__, __LINE__); \
    *((unsigned char *)0) = 0;                                         \
  } while (0);

#define SET_VALUE_BITS(x, val, start, bits) \
  (((x) & ~(((1 << (bits)) - 1) << (start))) | ((val) << (start)))

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define SWAP(type, a, b) \
  do {                   \
    type t;              \
    t = a;               \
    a = b;               \
    b = t;               \
  } while (0);

#define IS_BIT_SET(value, bit) (value & (1 << bit))

#define UNUSED_PARAM(param) ((void)(param))
#define __unused__ __attribute__((unused))

uint32_t sensord_popcount_32(uint32_t x);
uint32_t sensord_popcount_64(uint64_t x);

#ifdef __cplusplus
}
#endif

#endif
