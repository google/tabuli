/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/vreg.h"
#include "hardware/structs/systick.h"
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

#define NUM_CHANNELS 3

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(420000, true);

  for (uint32_t i = 0; i < 16; ++i) {
    gpio_init(i);
    gpio_set_dir(i, GPIO_OUT);
    gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
  }
  //gpio_set_drive_strength(0, GPIO_DRIVE_STRENGTH_12MA);

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

  uint32_t bit0 = 0;
  uint32_t bit1 = 0;
  uint32_t bit2 = 0;
  uint32_t bit3 = 0;
  uint32_t bit4 = 0;
  uint32_t prev_pin = 0;

  const uint32_t kBits4X[4] = {404, 276, 276, 0?268: 1};

  const uint32_t kSubSamples[9] = {0, 1586, 905, 542, kBits4X[3], 310, 260, 223, 158};

  systick_hw->rvr = 0xFFFFFF;
  systick_hw->csr = 5; // enable + no-int + cpu-clk

  while (true) {
    //set[2] = 1 << 15;
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

#define STEP(I)                                                                \
  {                                                                            \
    q##I -= vol##I;                                                            \
    mask = q##I >> (32 - SAMPLE_BITS);                                         \
    q##I += mask;                                                              \
    quantized |= mask & (1u << I);                                             \
  }

    for (uint32_t j = 0; j < kSubSamples[NUM_CHANNELS]; ++j) {
      systick_hw->cvr = 0;
      uint32_t quantized = 0;
      uint32_t mask;
      if (NUM_CHANNELS > 0) {
        STEP(0);
      }
      if (NUM_CHANNELS > 1) {
        STEP(1);
      }
      if (NUM_CHANNELS > 2) {
        STEP(2);
      }
      if (NUM_CHANNELS > 3) {
        STEP(3);
      }
      if (NUM_CHANNELS > 4) {
        STEP(4);
      }
      if (NUM_CHANNELS > 5) {
        STEP(5);
      }
      if (NUM_CHANNELS > 6) {
        STEP(6);
      }
      if (NUM_CHANNELS > 7) {
        STEP(7);
      }
      uint32_t want_toggle = quantized ^ prev_pin;
      uint32_t carry0 = want_toggle & bit0;
      bit0 ^= want_toggle;
      uint32_t carry1 = carry0 & bit1;
      bit1 ^= carry0;
      uint32_t carry2 = carry1 & bit2;
      bit2 ^= carry1;
      uint32_t carry3 = carry2 & bit3;
      bit3 ^= carry2;
      uint32_t carry4 = carry2 & bit4;
      bit4 ^= carry2;

      uint32_t toggle = carry3;
      prev_pin ^= toggle;
      set[2] = toggle;

      uint32_t tick1 = systick_hw->cvr;
      uint32_t delta = 0xFFFFFF - tick1; // ticks are decreasing
      uint32_t too_longA = ((delta - 51) >> 31) & 1;
      uint32_t too_longB = ((delta - 52) >> 31) & 1;
      uint32_t too_longC = ((delta - 53) >> 31) & 1;
      uint32_t too_longD = ((delta - 54) >> 31) & 1;
      set[2] = (too_longA << 8) | (too_longB << 9) | (too_longC << 10) | (too_longD << 11);
    }
// 3ch 45 instr -> 52 clk
/*
100003ba:	9b01      	ldr	r3, [sp, #4]
100003bc:	1ad2      	subs	r2, r2, r3
100003be:	9b02      	ldr	r3, [sp, #8]
100003c0:	0e15      	lsrs	r5, r2, #24
100003c2:	1ac9      	subs	r1, r1, r3
100003c4:	0e0b      	lsrs	r3, r1, #24
100003c6:	469b      	mov	fp, r3
100003c8:	465f      	mov	r7, fp
100003ca:	9b03      	ldr	r3, [sp, #12]
100003cc:	4037      	ands	r7, r6
100003ce:	1ae4      	subs	r4, r4, r3
100003d0:	0e23      	lsrs	r3, r4, #24
100003d2:	3602      	adds	r6, #2
100003d4:	18e4      	adds	r4, r4, r3
100003d6:	4033      	ands	r3, r6
100003d8:	2601      	movs	r6, #1
100003da:	433b      	orrs	r3, r7
100003dc:	1952      	adds	r2, r2, r5
100003de:	4035      	ands	r5, r6
100003e0:	432b      	orrs	r3, r5
100003e2:	464d      	mov	r5, r9
100003e4:	9f00      	ldr	r7, [sp, #0]
100003e6:	406b      	eors	r3, r5
100003e8:	003d      	movs	r5, r7
100003ea:	405f      	eors	r7, r3
100003ec:	401d      	ands	r5, r3
100003ee:	0003      	movs	r3, r0
100003f0:	9700      	str	r7, [sp, #0]
100003f2:	4667      	mov	r7, ip
100003f4:	402b      	ands	r3, r5
100003f6:	4068      	eors	r0, r5
100003f8:	4665      	mov	r5, ip
100003fa:	405f      	eors	r7, r3
100003fc:	401d      	ands	r5, r3
100003fe:	46bc      	mov	ip, r7
10000400:	4643      	mov	r3, r8
10000402:	4647      	mov	r7, r8
10000404:	402b      	ands	r3, r5
10000406:	406f      	eors	r7, r5
10000408:	464d      	mov	r5, r9
1000040a:	4e1d      	ldr	r6, [pc, #116]	; (10000480 <main+0x174>)
1000040c:	405d      	eors	r5, r3
1000040e:	46a9      	mov	r9, r5
10000410:	4d16      	ldr	r5, [pc, #88]	; (1000046c <main+0x160>)
10000412:	6033      	str	r3, [r6, #0]
*/
  }
}
