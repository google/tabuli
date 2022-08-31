#include "hardware/structs/systick.h"
#include "hardware/vreg.h"
#include "pico/assert.h"
#include "pico/stdlib.h"
#include <stdint.h>

#define CONCAT(A, B) DO_CONCAT(A, B)
#define DO_CONCAT(A, B) A##B

#define SAMPLE_BITS 16
#define sample_t CONCAT(CONCAT(uint, SAMPLE_BITS), _t)

extern uint8_t payload_start;

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420
#define SAMPLING_FREQ 22050

#define CLK_PER_LOOP 147
//         |DIV|RAM|
// --------+---+---+
// 2x4ch-c1|110| 95|
// 2x5ch-c1|130|113|
// 2x6ch-c1|148|133|
// 2x7ch-c1|   |147|

#define MAX_TICK 0xFFFFFF
#define DEBUG_CLK 1

//#define NUM_CHANNELS 8
// Actually 2x more
#define NUM_CHANNELS 7

#define STEP(I)                                                                \
  {                                                                            \
    q##I += sound[cntr + I];                                                   \
    quantized |= (q##I & 0x80008000) >> (15 - I);                              \
    q##I &= 0x7FFF7FFF;                                                        \
  }

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);

  for (uint32_t i = 0; i < 16; ++i) {
    gpio_init(i);
    gpio_set_dir(i, GPIO_OUT);
    // gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
  }
  // gpio_set_drive_strength(0, GPIO_DRIVE_STRENGTH_12MA);

  uint32_t *sound = (uint32_t*)&payload_start;

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
    cntr += clk & 1;
    cntr &= 0x7FFF;
    uint32_t quantized = 0;
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
    const uint32_t kSoundPinsMask = (1 << (NUM_CHANNELS * 2)) - 1;
    quantized = (quantized | (quantized >> (16 - NUM_CHANNELS))) & kSoundPinsMask;
    uint32_t want_toggle = quantized ^ prev_pin;
    uint32_t carry0 = want_toggle & bit0;
    bit0 ^= want_toggle;
    uint32_t carry1 = carry0 & bit1;
    bit1 ^= carry0;

    uint32_t toggle = carry1;
    prev_pin ^= toggle;
    *togl = toggle;

    if (DEBUG_CLK) {
      uint32_t tick = systick_hw->cvr;
      uint32_t delta = MAX_TICK - tick;
      uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
      *togl = in_time << 15;
      systick_hw->cvr = 0;
    }
  }
}
