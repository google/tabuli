#include "hardware/structs/systick.h"
#include "hardware/vreg.h"
#include "pico/assert.h"
#include "pico/stdlib.h"
#include <stdint.h>

#define CONCAT(A, B) DO_CONCAT(A, B)
#define DO_CONCAT(A, B) A##B

#define SAMPLE_BITS 8
#define sample_t CONCAT(CONCAT(uint, SAMPLE_BITS), _t)

extern uint8_t payload_start;
extern uint8_t payload_end;

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420
#define SAMPLING_FREQ 22050

#define CLK_PER_LOOP 181
// 8ch-c1:181
// 8ch-c4:189
// 8ch-wt:166

#define MAX_TICK 0xFFFFFF
#define DEBUG_CLK 0

#define NUM_CHANNELS 8

#define STEP(I)                                                                \
  {                                                                            \
    q##I -= sound[cntr + I];                                                   \
    mask = (int32_t)(q##I) >> 31;                                              \
    q##I += mask & 0x100;                                                      \
    quantized |= mask & (1u << I);                                             \
  }

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);

  for (uint32_t i = 0; i < 16; ++i) {
    gpio_init(i);
    gpio_set_dir(i, GPIO_OUT);
    //gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
  }
  // gpio_set_drive_strength(0, GPIO_DRIVE_STRENGTH_12MA);

  sample_t *sound = &payload_start;
  sample_t *sound_end = &payload_end;
  uint32_t num_samples = sound_end - sound;

  // io_wo_32 *set = &sio_hw->gpio_set;
  // io_wo_32 *clr = &sio_hw->gpio_clr;
  io_wo_32 *togl = &sio_hw->gpio_togl;

  uint32_t cntr = 0;
  uint32_t sampler = 0;
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
  uint32_t bit5 = 0;
  uint32_t bit6 = 0;
  uint32_t prev_pin = 0;

  if (DEBUG_CLK) {
    systick_hw->rvr = MAX_TICK;
    systick_hw->csr = 5; // enable + no-int + cpu-clk
  }

  const uint32_t kClkPerSecond = CPU_FREQ_MHZ * 1000000;
  const uint32_t kSamplePlace = SAMPLING_FREQ * CLK_PER_LOOP;

  while (true) {
    sampler += kSamplePlace;
    // ~0 if need to go forward, 0 otherwise
    uint32_t clk = -((kClkPerSecond - sampler) >> 31);
    sampler -= clk & kClkPerSecond;
    // Currently only 2 channels.
    cntr += clk & 2;
    // 0 when need to wrap, 1 otherwise
    uint32_t not_wrap = (cntr - num_samples) >> 31;
    cntr &= -not_wrap; // 0 only when need to wrap, no-op otherwise
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
    uint32_t carry4 = carry3 & bit4;
    bit4 ^= carry3;

    uint32_t toggle = carry1;
    prev_pin ^= toggle;
    *togl = toggle;

    if (DEBUG_CLK) {
      uint32_t tick = systick_hw->cvr;
      uint32_t delta = MAX_TICK - tick;
      uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
      *togl = in_time << 8;
      systick_hw->cvr = 0;
    }
  }
}
