#include "hardware/gpio.h"
#include "hardware/structs/systick.h"
#include "hardware/vreg.h"
#include "pico/assert.h"
#include "pico/stdlib.h"
#include <hardware/pio.h>
#include <stdint.h>

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420

/*
phase:
    nop
entry:
    pull
    mov x, osr
    pull
again:
    out isr, 1
    in x, 1
    mov pins, isr
    nop
    jmp !osre
    jmp phase
 */

// Sampling frequency: 44100
// Number of channels: 16
// FIFO length: 8
// FIFO bits: 8 * 32 = 256
// Number of channels per PIO: 2
// Bits per channel ber PIO: 256 / 2 = 128
// Clk per PIO bit: 7
// Clk per loop: 128 * 7 = 896
// Avg clk per channel per loop: 896 / 16 = 8 * 7 = 56
// Oversampling: 420M / 44100 / 896 ~= 10.63
// Sample step per loop: 44100 * 896 / 420M ~= 0.094
// -//- * 65536: 6165 + 1959/3125

uint32_t kSampleStepInt = 6165;
uint32_t kSampleStepRem = 1959;
uint32_t kSampleStepDiv = 3125;

#define MAX_TICK 0xFFFFFF
#define DEBUG_CLK 1
#define CLK_PER_LOOP (1536 - 112)

static uint32_t sample_base[2048];
static uint32_t precomputed1[128 * 8];
static uint32_t precomputed2[128 * 8];

void __attribute__((noinline))
merge(uint32_t *to, uint32_t *from1, uint32_t *from2) {
  for (uint32_t j = 0; j < 8; ++j) {
    to[j * 8] = from1[j] | from2[j];
  }
}

int main() {
  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
    // Wait until clock is stable.
    for (uint32_t i = 0; i < 50000; ++i) {
      NOP;
    }
  }

  gpio_init(16);
  gpio_set_dir(16, GPIO_OUT);

  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);

  // gpio_set_slew_rate(x, GPIO_SLEW_RATE_FAST);
  // gpio_set_drive_strength(x, GPIO_DRIVE_STRENGTH_2MA);

  io_wo_32 *set = &sio_hw->gpio_set;
  io_wo_32 *clr = &sio_hw->gpio_clr;
  // io_wo_32 *togl = &sio_hw->gpio_togl;
  *set = 1 << 25;

  if (DEBUG_CLK) {
    systick_hw->rvr = MAX_TICK;
    systick_hw->csr = 5; // enable + no-int + cpu-clk
  }

  // One item per 2 channels.
  uint32_t qs[8] = {0};
  // 8 items for each of 8 PIO SM.
  uint32_t pio_out[64] = {0};
  uint32_t sample_pos = 0;
  uint32_t sample_tail = 0;
  // uint32_t* precomputed1 = 0;
  // uint32_t* precomputed2 = 0;

  while (true) {
    sample_pos += kSampleStepInt;
    sample_tail += kSampleStepRem;
    if (sample_tail > kSampleStepDiv) {
      sample_tail -= kSampleStepDiv;
      sample_pos++;
    }

    uint32_t next_mul = sample_pos & 0xFFFF;
    uint32_t mul = 0x10000 - next_mul;

    // 1 << 16 is sample step multiplier
    // 1 << 3 stands for number of channel-pairs
    // Each scratch memory bank is 4096 bytes = 128 * (2 * 16)
    // Scratch banks are mapped without a gap, giving 256 samples in total.
    // TODO: >> & << ?
    uint32_t sample_idx = (sample_pos >> (16 - 3)) & (0xFF << 3);
    uint32_t next_sample_idx = (sample_idx + 8) & (0xFF << 3);
    // uint32_t* sample_base = 0;
    uint32_t *sample = sample_base + sample_idx;
    uint32_t *next_sample = sample_base + next_sample_idx;

    for (uint32_t i = 0; i < 8; ++i) {
      uint32_t val = sample[i];
      uint32_t next_val = next_sample[i];
      uint32_t q = qs[i];

      uint32_t ch1 = val & 0xFFFF;
      uint32_t next_ch1 = next_val & 0xFFFF;
      uint32_t ch1_l = ch1 * mul;
      uint32_t ch1_r = next_ch1 * next_mul;
      uint32_t ch1_current = (ch1_l + ch1_r) >> 16;
      uint32_t ch1_tmp = q & 0xFFFF + ch1_current;

      uint32_t ch2 = val >> 16;
      uint32_t next_ch2 = next_val >> 16;
      uint32_t ch2_l = ch2 * mul;
      uint32_t ch2_r = next_ch2 * next_mul;
      uint32_t ch2_current = (ch2_l + ch2_r) >> 16;
      uint32_t ch2_tmp = (q >> 16) + ch2_current;

      qs[i] = (ch1_tmp & 0x1FF) | ((ch2_tmp & 0x1FF) << 16);

      uint32_t *ch1_src = precomputed1 + ((ch1_tmp >> 9) << 3);
      uint32_t *ch2_src = precomputed2 + ((ch2_tmp >> 9) << 3);
      // for (uint32_t j = 0; j < 8; ++j) {
      //   pio_out[j * 8 + i] = ch1_src[j] | ch2_src[j];
      // }
      //merge(pio_out + i, ch1_src, ch2_src);
    }

    for (uint32_t j = 0; j < 8; ++j) {
      // while (pio_sm_is_tx_fifo_full(pio0, 0)) {
      //  wait for FIFO; all SM should become ready since they are synced
      //}
      register uint32_t *src = pio_out + 8 * j;
      pio_sm_put(pio0, 0, src[0]);
      pio_sm_put(pio0, 1, src[1]);
      pio_sm_put(pio0, 2, src[2]);
      pio_sm_put(pio0, 3, src[3]);
      pio_sm_put(pio1, 0, src[4]);
      pio_sm_put(pio1, 1, src[5]);
      pio_sm_put(pio1, 2, src[6]);
      pio_sm_put(pio1, 3, src[7]);
    }

    if (DEBUG_CLK) {
      uint32_t tick = systick_hw->cvr;
      uint32_t delta = MAX_TICK - tick;
      uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
      *(in_time ? set : clr) = 1 << 16;
      systick_hw->cvr = 0;
    }
  }
}
