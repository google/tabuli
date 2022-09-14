#include "hardware/gpio.h"
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

#define CLK_PER_LOOP 135
//         |DIV|RAM|
// --------+---+---+
// 2x1ch-c1|   | 48|
// 2x4ch-c1|110| 95|
// 2x5ch-c1|130|113|
// 2x6ch-c1|148|133|
// 2x7ch-c1|   |147|

#define MAX_TICK 0xFFFFFF
#define DEBUG_CLK 1

//#define NUM_CHANNELS 8
// Actually 2x more
#define NUM_CHANNELS 1

#define STEP(I)                                                                \
  {                                                                            \
    q##I += sound[cntr + I];                                                   \
    quantized |= (q##I & 0x80008000) >> (15 - I);                              \
    q##I &= 0x7FFF7FFF;                                                        \
  }

#define SD4 1

static uint8_t dither[256] = {
    1, 0, 2, 2, 2, 1, 0, 2, 1, 1, 2, 0, 2, 0, 1, 2, 2, 1, 1, 1, 0, 1, 0, 1,
    2, 0, 1, 0, 1, 1, 2, 0, 1, 2, 2, 0, 2, 1, 2, 1, 2, 0, 0, 1, 0, 1, 2, 1,
    2, 2, 1, 2, 1, 2, 2, 1, 1, 1, 1, 2, 1, 0, 2, 2, 1, 0, 0, 0, 2, 0, 2, 0,
    2, 1, 0, 2, 0, 0, 2, 2, 0, 2, 1, 0, 2, 0, 2, 0, 2, 2, 0, 2, 1, 2, 2, 0,
    0, 1, 2, 0, 0, 1, 1, 0, 0, 2, 1, 0, 1, 2, 2, 1, 1, 0, 2, 2, 0, 1, 1, 1,
    1, 2, 2, 1, 0, 0, 1, 1, 1, 0, 0, 2, 1, 0, 2, 0, 2, 1, 0, 1, 2, 1, 0, 2,
    2, 2, 1, 0, 0, 1, 1, 1, 1, 2, 0, 1, 2, 2, 0, 0, 2, 1, 2, 2, 2, 1, 0, 2,
    2, 2, 1, 0, 1, 2, 1, 1, 1, 0, 1, 2, 2, 1, 0, 1, 2, 2, 0, 2, 0, 2, 2, 0,
    0, 1, 2, 2, 1, 1, 1, 2, 1, 0, 2, 0, 1, 1, 1, 0, 2, 0, 1, 0, 0, 1, 2, 2,
    0, 2, 2, 0, 2, 2, 0, 2, 1, 0, 1, 1, 2, 1, 1, 2, 2, 1, 0, 0, 0, 2, 0, 2,
    0, 1, 1, 2, 1, 1, 2, 1, 2, 0, 2, 2, 0, 0, 2, 0};

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);

  for (uint32_t i = 0; i < 17; ++i) {
    gpio_init(i);
    gpio_set_dir(i, GPIO_OUT);
    //gpio_set_slew_rate(i, GPIO_SLEW_RATE_FAST);
    //gpio_set_drive_strength(0, GPIO_DRIVE_STRENGTH_2MA);
  }

  uint32_t *sound = (uint32_t *)&payload_start;

  uint32_t silo[16] = {0};

  // io_wo_32 *set = &sio_hw->gpio_set;
  // io_wo_32 *clr = &sio_hw->gpio_clr;
  io_wo_32 *togl = &sio_hw->gpio_togl;

#if SD4
  int32_t w[4] = {0, 0, 0, 0};
  int32_t scale = 1;
  int32_t vmin_04 = -32767 * scale;
  int32_t pos_error_04 = 65534 * scale;
#endif

  uint32_t dither_cntr = 0;

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

    uint16_t *snd16 = (uint16_t *)(sound + cntr);
    uint16_t vol16u = *snd16;
    int16_t vol16i = vol16u - 16384;
    int32_t vol = vol16i * 2;

#if SD4
    int32_t d = vmin_04 - vol;
    int32_t test = vmin_04 * 1024;
#endif

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

#if SD4
    int32_t wn = d + 4 * (w[0] + w[2]) - 6 * w[1] - w[3];
    int32_t etmp =
        -3035 * w[0] + 3477 * w[1] - 1809 * w[2] + 359 * w[3] + 1024 * wn;
    w[3] = w[2];
    w[2] = w[1];
    w[1] = w[0];
    w[0] = wn;
    int32_t chk = etmp < test ? 1 : 0;
    w[0] += (-chk & pos_error_04);
#endif

#if NUM_CHANNELS > 0
    const uint32_t kSoundPinsMask = (1 << (NUM_CHANNELS * 2)) - 1;
    quantized =
        (quantized | (quantized >> (16 - NUM_CHANNELS))) & kSoundPinsMask;
#endif
    // uint32_t silo_idx = dither[dither_cntr++ & 0x1];
    // silo[0] = quantized;
    // quantized = silo[silo_idx];
    // silo[silo_idx] = silo[0];

    quantized = (quantized & 0x3);
    //quantized |= (ch << 14);
#if SD4
    quantized |= (chk << 15);
#endif

    uint32_t want_toggle = quantized ^ prev_pin;
    // uint32_t carry0 = want_toggle & bit0;
    // bit0 ^= want_toggle;
    // uint32_t carry1 = carry0 & bit1;
    // bit1 ^= carry0;

    uint32_t toggle = want_toggle;
    prev_pin ^= toggle;
    *togl = toggle;

    if (DEBUG_CLK) {
      uint32_t tick = systick_hw->cvr;
      uint32_t delta = MAX_TICK - tick;
      uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
      *togl = in_time << 16;
      systick_hw->cvr = 0;
    }
  }
}
