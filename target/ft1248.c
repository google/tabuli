#include "display.h"

#include "ft1248.pio.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <hardware/pio.h>

#include <stdint.h>

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420

void ft1248_program_init(void) {
  PIO pio = pio0;
  uint32_t sm = 0;

  pio_clear_instruction_memory(pio);
  pio_add_program_at_offset(pio, &ft1248_program, 0);

  const uint32_t data0_pin = 0;
  // const uint32_t data7_pin = 7;
  const uint32_t miso_pin = 8;
  const uint32_t ss_pin = 9;
  const uint32_t clk_pin = 10;

  pio_sm_config c = pio_get_default_sm_config();

  sm_config_set_out_pins(&c, data0_pin, 8);
  sm_config_set_in_pins(&c, data0_pin);
  sm_config_set_sideset_pins(&c, ss_pin);
  sm_config_set_sideset(&c, 2, false, false);
  sm_config_set_clkdiv_int_frac(&c, /* div_int */ 1, /* div_frac */ 0);
  sm_config_set_wrap(&c, ft1248_wrap_target, ft1248_wrap);
  sm_config_set_jmp_pin(&c, miso_pin);
  // TODO: shift-right?
  sm_config_set_in_shift(&c, /* shift_right */ false, /* autopush */ true,
                         /* push threshold */ 32);
  // sm_config_set_out_shift(&c, shift_right, autopull, pull_threshold);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  // sm_config_set_out_special(&c, sticky, has_enable_pin, enable_pin_index);
  // sm_config_set_mov_status(&c, status_sel, status_n);

  // Setup read command.
  pio_sm_set_pins_with_mask(pio, sm, 0x40u << data0_pin, 0xFF << data0_pin);
  // All data pins are in by default.
  // TODO: pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  pio_sm_set_pindirs_with_mask(pio, sm, 0, 0xFFu << data0_pin);

  // MISO is always in.
  pio_sm_set_pindirs_with_mask(pio, sm, 0, 1u << miso_pin);

  // Setup idle device state: SS_N high, CLK low.
  pio_sm_set_pins_with_mask(pio, sm, 1u << ss_pin, 3u << ss_pin);
  // Control pins are always out.
  pio_sm_set_pindirs_with_mask(pio, sm, 3u << ss_pin, 3u << ss_pin);

  for (uint32_t i = data0_pin; i <= clk_pin; ++i) {
    pio_gpio_init(pio, i);
  }

  pio_sm_init(pio, sm, ft1248_offset_entry_point, &c);
}

static void print_hex(char* out, uint32_t value) {
  for (int32_t i = 7; i >= 0; i--) {
    uint32_t digit = value & 0xF;
    out[i] = digit + ((digit < 10) ? '0' : ('A' - 10));
    value >>= 4;
  }
}

void core1_main() {
  display_init();
  display_print(0, 0, "DEAD");
  display_print(4, 6, "BEEF");
  display_print(8, 12, "CAFE");
  display_print(12, 18, "BABE");
  display_flush();

  char txt[33] = "01234567:01234567/01234567      ";

  uint32_t next_line = 0;
  uint32_t index = 0;
  multicore_fifo_push_blocking(0); // signal we are ready
  while (1) {
    if (!multicore_fifo_rvalid()) {
      continue;
    }
    if (index == 0) {
      display_clear();
    }
    index++;
    uint32_t iter = multicore_fifo_pop_blocking();
    uint32_t errors = multicore_fifo_pop_blocking();
    uint32_t bytes = multicore_fifo_pop_blocking();
    txt[8] = index & 1 ? ':' : ' ';
    print_hex(txt, iter);
    print_hex(txt + 9, errors);
    print_hex(txt + 18, bytes);
    display_print(0, next_line * 6, txt);
    display_flush();
    next_line++;
    if (next_line == 9) {
      next_line = 0;
    }
    multicore_fifo_push_blocking(0); // signal we are ready
  }
}

void core0_main() {
  if (CPU_FREQ_MHZ != 125) {
    for (uint32_t i = 0; i < 20 * 1000 * 1000; ++i) {
      NOP;
    }
  }

  PIO pio = pio0;
  uint32_t sm = 0;

  uint32_t iter = 0;
  uint32_t errors = 0;
  uint32_t bytes = 0;
  while (!multicore_fifo_rvalid()) {
    NOP;
  }

  pio_sm_set_enabled(pio, sm, true);

  while (1) {
    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
      uint32_t vals = pio_sm_get(pio, sm);
      bytes += 4;
      iter += vals & 0xFF;
      vals >>= 8;
      iter += vals & 0xFF;
      vals >>= 8;
      iter += vals & 0xFF;
      vals >>= 8;
      iter += vals;
    }
    // Expected sum for 64MiB sequence: 0xfe00_0000
    if (multicore_fifo_rvalid()) {
      (void)multicore_fifo_pop_blocking();
      multicore_fifo_push_blocking(iter);
      multicore_fifo_push_blocking(errors);
      multicore_fifo_push_blocking(bytes);
    }
  }
}

uint32_t kCore0Ready = 0xFEEDBAC0;
uint32_t kCore1Ready = 0xFEEDBAC1;

void core1_start() {
  uint32_t test = multicore_fifo_pop_blocking();
  if (test == kCore0Ready) {
    multicore_fifo_push_blocking(kCore1Ready);
    core1_main();
  }
  while (1) {
    tight_loop_contents();
  }
}

int main() {
  ft1248_program_init();

  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
    // Wait until clock is stable.
    for (uint32_t i = 0; i < 50000; ++i) {
      NOP;
    }
  }
  multicore_launch_core1(core1_start);

  multicore_fifo_push_blocking(kCore0Ready);
  uint32_t test = multicore_fifo_pop_blocking();
  if (test == kCore1Ready) {
    core0_main();
  }
  while (1) {
    tight_loop_contents();
  }
}