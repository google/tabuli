#include "sound.pio.h"

#include "hardware/gpio.h"
#include "hardware/structs/systick.h"
#include "hardware/vreg.h"
#include "pico/assert.h"
#include "pico/stdlib.h"

#include <hardware/pio.h>
#include <stdint.h>

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420

void sound_program_init(uint32_t pio_n) {
  PIO pio = (pio_n == 0) ? pio0 : pio1;
  uint32_t entry_point =
      (pio_n == 0) ? sound_offset_entry_point0 : sound_offset_entry_point1;
  pio_clear_instruction_memory(pio);
  pio_add_program_at_offset(pio, &sound_program, 0);

  pio_sm_config c = pio_get_default_sm_config();

  // sm_config_set_in_pins(&c, data0_pin);
  // sm_config_set_sideset_pins(&c, ss_pin);
  // sm_config_set_sideset(&c, 2, false, false);
  sm_config_set_clkdiv_int_frac(&c, /* div_int */ 1, /* div_frac */ 0);
  sm_config_set_wrap(&c, sound_wrap_target, sound_wrap);
  // sm_config_set_jmp_pin(&c, miso_pin);
  sm_config_set_in_shift(&c, /* shift_right */ false, /* autopush */ false,
                         /* push threshold */ 32);
  sm_config_set_out_shift(&c, true, /* autopull */ false,
                          /* pull_threshold */ 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  // sm_config_set_out_special(&c, sticky, has_enable_pin, enable_pin_index);
  // sm_config_set_mov_status(&c, status_sel, status_n);

  for (uint32_t sm = 0; sm < 4; ++sm) {
    uint32_t out_pins = pio_n * 8 + 2 * sm;
    sm_config_set_out_pins(&c, out_pins, 2);
    // Clear pins.
    pio_sm_set_pins_with_mask(pio, sm, 0, 3 << out_pins);
    // Set direction
    pio_sm_set_consecutive_pindirs(pio, sm, out_pins, 2, true);

    for (uint32_t i = 0; i <= 2; ++i) {
      pio_gpio_init(pio, out_pins + i);
    }

    pio_sm_init(pio, sm, entry_point, &c);
  }
}

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
#define CLK_PER_LOOP (896 - 20)

static uint16_t sample_base[4096];
static uint32_t precomputed[129 * 4];

void __attribute__((noinline))
merge(uint32_t *to, uint32_t *from1, uint32_t *from2) {
  for (uint32_t j = 0; j < 8; ++j) {
    to[j * 8] = from1[j] | from2[j];
  }
}

void main_loop();

int main() {
  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
    // Wait until clock is stable.
    for (uint32_t i = 0; i < 5000000; ++i) {
      NOP;
    }
  }

  gpio_init(16);
  gpio_set_dir(16, GPIO_OUT);

  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);

  // gpio_set_slew_rate(x, GPIO_SLEW_RATE_FAST);
  // gpio_set_drive_strength(x, GPIO_DRIVE_STRENGTH_2MA);

  sound_program_init(0);
  sound_program_init(1);

  pio0->ctrl = 0xF;
  pio1->ctrl = 0xF; // TODO: sync start
  // pio_set_sm_mask_enabled(pio0, 0xF, true);
  // pio_set_sm_mask_enabled(pio1, 0xF, true);
  main_loop();
}

void main_loop() {
  uint16_t bank[16] = {0};
  uint16_t qs[16] = {0};
  uint32_t sample_pos = 0;
  uint32_t sample_tail = 0;

#if DEBUG_CLK
  io_wo_32 *set = &sio_hw->gpio_set;
  io_wo_32 *clr = &sio_hw->gpio_clr;
  io_wo_32 *togl = &sio_hw->gpio_togl;
  systick_hw->rvr = MAX_TICK;
  systick_hw->csr = 5; // enable + no-int + cpu-clk
#endif

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
    // 1 << 4 stands for number of channels
    // Each scratch memory bank is 4096 bytes = 128 * (2 * 16)
    // Scratch banks are mapped without a gap, giving 256 samples in total.
    // TODO: remember about tail copying!
    uint32_t sample_idx = (sample_pos >> (16 - 4)) & (0xFF << 4);
    uint16_t *sample = sample_base + sample_idx;

#pragma GCC unroll 16
    for (uint32_t i = 0; i < 16; ++i) {
      //  2
      uint32_t tmp1 = sample[i + 16]; // fixed offset after unroll
      // 1
      tmp1 *= mul;
      // 2
      uint32_t tmp2 = sample[i]; // fixed offset after unroll
      // 1
      tmp2 *= next_mul;
      // 1
      tmp1 += tmp2;
      // 1
      tmp1 >>= 16;
      // 2
      tmp2 = qs[i]; // on stack
      // 1
      // tmp1 += tmp2;
      // Alas, GCC is too buggy to do that!
      asm(".thumb\n\t"
          ".syntax unified\n\t"
          "adds %0, %1, %2"
          : "=l"(tmp1)
          : "0"(tmp1), "l"(tmp2));
      // 1
      tmp2 = tmp1 << (32 - 9);
      // 1
      tmp2 >>= (32 - 9);
      // 2
      qs[i] = tmp2; // on stack
      // 1
      tmp1 >>= 9;
      // 1
      tmp1 <<= 4;
      // 2
      bank[i] = tmp1; // on stack
    }

    io_wo_32 *pio0txf = pio0->txf;
    io_wo_32 *pio1txf = pio1->txf;
#pragma GCC unroll 4
    for (uint32_t j = 0; j < 4; ++j) {
      const uint32_t *base = precomputed + j;
      pio0txf[0] = base[bank[0]];
      pio0txf[0] = base[bank[1]];
      pio0txf[1] = base[bank[2]];
      pio0txf[1] = base[bank[3]];
      pio0txf[2] = base[bank[4]];
      pio0txf[2] = base[bank[5]];
      pio0txf[3] = base[bank[6]];
      pio0txf[3] = base[bank[7]];
      pio1txf[0] = base[bank[8]];
      pio1txf[0] = base[bank[9]];
      pio1txf[1] = base[bank[10]];
      pio1txf[1] = base[bank[11]];
      pio1txf[2] = base[bank[12]];
      pio1txf[2] = base[bank[13]];
      pio1txf[3] = base[bank[14]];
      pio1txf[3] = base[bank[15]];
    }

    while (pio_sm_is_tx_fifo_full(pio0, 0)) {
      // wait for FIFO; all SM should become ready since they are synced
    }

#if DEBUG_CLK
    uint32_t tick = systick_hw->cvr;
    uint32_t delta = MAX_TICK - tick;
    uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
    *(in_time ? set : clr) = 1 << 25;
    systick_hw->cvr = 0;
#endif
  }
}
