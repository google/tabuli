/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#define R2(X)                                                                  \
  X;                                                                           \
  X;
#define R4(X)                                                                  \
  R2(X);                                                                       \
  R2(X);
#define R8(X)                                                                  \
  R4(X);                                                                       \
  R4(X);
#define R16(X)                                                                 \
  R8(X);                                                                       \
  R8(X);
#define R32(X)                                                                 \
  R16(X);                                                                      \
  R16(X);
#define R64(X)                                                                 \
  R32(X);                                                                      \
  R32(X);

int main() {
  const uint PIN = 15;
  gpio_init(PIN);
  gpio_set_dir(PIN, GPIO_OUT);

  io_wo_32 *set = &sio_hw->gpio_set;
  uint32_t pin = 1 << PIN;

  const uint32_t FREQ = 44100;

  const uint32_t LUT_BITS = 14;
  const uint32_t LUT_LEN = 1 << LUT_BITS;
  const uint32_t SHIFT = 32 - LUT_BITS;
  const uint32_t CLK_PER_STEP = 8;
  const uint32_t NUM_STEPS = 32;
  const uint32_t CLK_PER_LOAD = 15; // between 14 and 15; but I'd expect 10!
  const uint32_t CLK_PER_LOOP = CLK_PER_STEP * NUM_STEPS + CLK_PER_LOAD;
  const uint32_t CLK_PER_SECOND = 125000000; // 125MHz
  // FREQ Hz === ADD * LOOP_PER_SECOND == FREQ * 2^32
  // ADD = 2^32 * FREQ / (CLK_PER_SECOND / CLK_PER_LOOP)
  // ADD = 2^32 * FREQ * CLK_PER_LOOP / CLK_PER_SECOND
  const uint32_t ADD = (4294967296.0 * FREQ * CLK_PER_LOOP) / CLK_PER_SECOND;

  uint32_t cntr2 = 0;
  uint32_t q = 0;
  while (true) {
    cntr2 += ADD;
    set[0] = pin;
    asm volatile("" ::: "memory");
    uint32_t vol = sin[cntr2 >> SHIFT];
    uint32_t mask;

#define WORK                                                                   \
  q -= vol;                                                                    \
  mask = q >> 16;                                                              \
  q += mask;                                                                   \
  set[mask & 1] = pin;                                                         \
  asm volatile("" ::: "memory");

    R32(WORK);
  }
}
