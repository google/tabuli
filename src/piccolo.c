/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"

#define R4(X)                                                                  \
  X;                                                                           \
  X;                                                                           \
  X;                                                                           \
  X;
#define R16(X)                                                                 \
  R4(X);                                                                       \
  R4(X);                                                                       \
  R4(X);                                                                       \
  R4(X);
#define R64(X)                                                                 \
  R16(X);                                                                      \
  R16(X);                                                                      \
  R16(X);                                                                      \
  R16(X);
#define R256(X)                                                                \
  R64(X);                                                                      \
  R64(X);                                                                      \
  R64(X);                                                                      \
  R64(X);

#define CONCAT(A, B) DO_CONCAT(A, B)
#define DO_CONCAT(A, B) A##B

#define SAMPLE_BITS 8
#define sample_t CONCAT(CONCAT(uint, SAMPLE_BITS), _t)

extern uint8_t payload_start;
extern uint8_t payload_end;

#define NOP asm volatile("nop");

#define SYNC 1

int main() {
  for (uint32_t i = 0; i < 8; ++i) {
    gpio_init(i);
    gpio_set_dir(i, GPIO_OUT);
  }

  sample_t *sound = &payload_start;
  sample_t *sound_end = &payload_end;
  uint32_t num_samples = sound_end - sound;

  io_wo_32 *set = &sio_hw->gpio_set;

  uint32_t cntr = 0;
  uint32_t q0 = 0;
  uint32_t q1 = 0;
  uint32_t q2 = 0;
  uint32_t q3 = 0;
  uint32_t q4 = 0;
  uint32_t q5 = 0;
  uint32_t q6 = 0;
  uint32_t q7 = 0;
  while (true) {
    cntr += 2;
    // 0 when need to wrap, 1 otherwise
    uint32_t not_wrap = (cntr - num_samples) >> 31;
    cntr &= -not_wrap; // 0 only when need to wrap, no-op otherwise
    uint32_t vol0 = sound[cntr];
    uint32_t vol1 = sound[cntr + 1];
    uint32_t vol2 = sound[cntr + 2];
    uint32_t vol3 = sound[cntr + 3];
    uint32_t vol4 = sound[cntr + 4];
    uint32_t vol5 = sound[cntr + 5];
    uint32_t vol6 = sound[cntr + 6];
    uint32_t vol7 = sound[cntr + 7];

    uint32_t prev_pin = 0;

    // Nch | SYNC | ASYNC |
    // ----+------+-------+
    //   1 |   11 |    12 | ~ 515 good
    //   2 |   20 |    20 | ~ 283 good
    //   3 |   32 |    30 | ~ 189 ok
    //   4 |   42 |    40 | ~ 135 almost ok
    //   5 |   48 |    51 | ~ 118 not so bad
    //   6 |   63 |    64 | ~ 90 bad
    //   7 |   75 |    75 | ~ 75 bad
    //   8 |   85 |    86 | ~ 66 bad

#define STEP(I)                                                                \
  q##I -= vol##I;                                                              \
  mask = q##I >> (32 - SAMPLE_BITS);                                           \
  q##I += mask;                                                                \
  if (SYNC) {                                                                  \
    pin |= mask & (1u << I);                                                   \
  } else {                                                                     \
    set[mask & 1] = (1u << I);                                                 \
  }

    for (uint32_t j = 0; j < 283; ++j) {
      uint32_t pin = 0;
      uint32_t mask;
      STEP(0);  STEP(1); // STEP(2); STEP(3);  STEP(4); STEP(5); STEP(6); STEP(7);
      if (SYNC) {
        set[2] = pin ^ prev_pin;
        prev_pin = pin;
      }
    }
    set[1] = 0xFF;
  }
}
