// SPDX-License-Identifier: Apache-2.0
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

#include "util_misc.h"

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

uint32_t sensord_popcount_32(uint32_t x) {
  const uint32_t m1 = 0x55555555;   // binary: 0101...
  const uint32_t m2 = 0x33333333;   // binary: 00110011..
  const uint32_t m4 = 0x0f0f0f0f;   // binary:  4 zeros,  4 ones ...
  const uint32_t m8 = 0x00ff00ff;   // binary:  8 zeros,  8 ones ...
  const uint32_t m16 = 0x0000ffff;  // binary: 16 zeros, 16 ones ...
  // const uint32_t hff = 0xffffffff; //binary: all ones
  // const uint32_t h01 = 0x01010101; //the sum of 256 to the power of
  // 0,1,2,3...

  x = (x & m1) + ((x >> 1) & m1);  // put count of each  2 bits into those  2
                                   // bits
  x = (x & m2) + ((x >> 2) & m2);  // put count of each  4 bits into those  4
                                   // bits
  x = (x & m4) + ((x >> 4) & m4);  // put count of each  8 bits into those  8
                                   // bits
  x = (x & m8) + ((x >> 8) & m8);  // put count of each 16 bits into those 16
                                   // bits
  x = (x & m16) +
      ((x >> 16) & m16);  // put count of each 32 bits into those 32 bits

  return x;
}

uint32_t sensord_popcount_64(uint64_t x) {
  const uint64_t m1 = 0x5555555555555555;   // binary: 0101...
  const uint64_t m2 = 0x3333333333333333;   // binary: 00110011..
  const uint64_t m4 = 0x0f0f0f0f0f0f0f0f;   // binary:	4 zeros,  4 ones ...
  const uint64_t m8 = 0x00ff00ff00ff00ff;   // binary:	8 zeros,  8 ones ...
  const uint64_t m16 = 0x0000ffff0000ffff;  // binary: 16 zeros, 16 ones ...
  const uint64_t m32 = 0x00000000ffffffff;  // binary: 32 zeros, 32 ones
  // const uint64_t hff = 0xffffffffffffffff; //binary: all ones
  // const uint64_t h01 = 0x0101010101010101; //the sum of 256 to the power of
  // 0,1,2,3...

  x = (x & m1) + ((x >> 1) & m1);  // put count of each  2 bits into those  2
                                   // bits
  x = (x & m2) + ((x >> 2) & m2);  // put count of each  4 bits into those  4
                                   // bits
  x = (x & m4) + ((x >> 4) & m4);  // put count of each  8 bits into those  8
                                   // bits
  x = (x & m8) + ((x >> 8) & m8);  // put count of each 16 bits into those 16
                                   // bits
  x = (x & m16) +
      ((x >> 16) & m16);  // put count of each 32 bits into those 32 bits
  x = (x & m32) +
      ((x >> 32) & m32);  // put count of each 64 bits into those 64 bits

  return x;
}