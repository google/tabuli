#include "display.h"
#include "ft1248.pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pspi.pio.h"
#include <hardware/pio.h>

#include <stdint.h>

#define USE_DISPLAY 1
#define FAKE_PUSH 0

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420
#define CPU_FREQ_KHZ (CPU_FREQ_MHZ * 1000)

uint32_t kCore0Ready = 0xFEEDBAC0;
uint32_t kCore1Ready = 0xFEEDBAC1;

#define RING_BUFFER_SIZE 0x8000
#define RING_BUFFER_MASK (RING_BUFFER_SIZE - 1)
static uint32_t ring_buffer[RING_BUFFER_SIZE];

void init_flash(void) {
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
}

void flash(void) {
  gpio_put(25, 1);
  sleep_ms(100);
  gpio_put(25, 0);
  sleep_ms(100);
}

void init_pio(void) {
  pio_clear_instruction_memory(pio0);
  uint32_t ft1248_offset = pio_add_program(pio0, &ft1248_program);
  pio_clear_instruction_memory(pio1);
  uint32_t pspi_offset = pio_add_program(pio1, &pspi_program);

  uint32_t pull_sm = 0;

  const uint32_t data0_pin = 1;
  // const uint32_t data7_pin = 8;
  const uint32_t miso_pin = 0;

  const uint32_t ss_pin = 9;
  const uint32_t clk_pin = 10;

  pio_sm_config pull_c = pio_get_default_sm_config();

  sm_config_set_out_pins(&pull_c, data0_pin, 8);
  sm_config_set_in_pins(&pull_c, data0_pin);
  // sm_config_set_in_pins(&pull_c, miso_pin);
  sm_config_set_sideset_pins(&pull_c, ss_pin);
  sm_config_set_sideset(&pull_c, 2, false, false);
  sm_config_set_clkdiv_int_frac(&pull_c, /* div_int */ 1, /* div_frac */ 0);
  sm_config_set_wrap(&pull_c, ft1248_offset + ft1248_wrap_target,
                     ft1248_offset + ft1248_wrap);
  sm_config_set_jmp_pin(&pull_c, miso_pin);
  sm_config_set_in_shift(&pull_c, /* shift_right */ true, /* autopush */ true,
                         /* push threshold */ 32);
  sm_config_set_out_shift(&pull_c, /* shift_right */ true, /* autopull */ false,
                          /* pull_threshold */ 32);
  sm_config_set_fifo_join(&pull_c, PIO_FIFO_JOIN_RX);
  // sm_config_set_out_special(&c, sticky, has_enable_pin, enable_pin_index);
  sm_config_set_mov_status(&pull_c, STATUS_RX_LESSTHAN, 7);

  // Setup read command.
  pio_sm_set_pins_with_mask(pio0, pull_sm, 0x40u << data0_pin,
                            0xFF << data0_pin);
  // All data pins are in by default.
  // TODO: pio_sm_set_consecutive_pindirs(pio0, sm, pin, 1, true);
  pio_sm_set_pindirs_with_mask(pio0, pull_sm, 0, 0xFFu << data0_pin);

  // MISO is always in.
  pio_sm_set_pindirs_with_mask(pio0, pull_sm, 0, 1u << miso_pin);

  // Setup idle device state: SS_N high, CLK low.
  pio_sm_set_pins_with_mask(pio0, pull_sm, 1u << ss_pin, 3u << ss_pin);
  // Control pins are always out.
  pio_sm_set_pindirs_with_mask(pio0, pull_sm, 3u << ss_pin, 3u << ss_pin);

  //----------------------------------------------------------------------------

  uint32_t push_sm = 1;
  const uint32_t num_mosi_pin = 15;
  const uint32_t cs_pin = 11;
  const uint32_t sclk_pin = 12;
  const uint32_t mosi0_pin = 13;
  const uint32_t mosi_last_pin = mosi0_pin + num_mosi_pin - 1;
  pio_sm_config push_c = pio_get_default_sm_config();

  sm_config_set_out_pins(&push_c, mosi0_pin, num_mosi_pin);
  // sm_config_set_in_pins(&push_c, data0_pin);
  sm_config_set_sideset_pins(&push_c, cs_pin);
  sm_config_set_sideset(&push_c, 2, false, false);
  sm_config_set_clkdiv_int_frac(&push_c, /* div_int */ 2, /* div_frac */ 0);
  sm_config_set_wrap(&push_c, pspi_offset + pspi_wrap_target,
                     pspi_offset + pspi_wrap);
  // sm_config_set_jmp_pin(&push_c, miso_pin);
  // sm_config_set_in_shift(&push_c, /* shift_right */ true, /* autopush */
  // true, /* push threshold */ 32);
  sm_config_set_out_shift(&push_c, /* shift_right */ true, /* autopull */ false,
                          /* pull_threshold */ 32);
  sm_config_set_fifo_join(&push_c, PIO_FIFO_JOIN_TX);
  // sm_config_set_out_special(&c, sticky, has_enable_pin, enable_pin_index);
  sm_config_set_mov_status(&push_c, STATUS_TX_LESSTHAN, 8);

  // All mosi pins are out.
  uint32_t mosi_mask = ((1 << num_mosi_pin) - 1);
  pio_sm_set_pins_with_mask(pio1, push_sm, 0, mosi_mask << mosi0_pin);
  pio_sm_set_pindirs_with_mask(pio1, push_sm, mosi_mask << mosi0_pin,
                               mosi_mask << mosi0_pin);
  // CS and SCLK are out as well.
  pio_sm_set_pins_with_mask(pio1, push_sm, 1 << cs_pin, 1 << cs_pin);
  pio_sm_set_pins_with_mask(pio1, push_sm, 0, 1 << sclk_pin);
  pio_sm_set_pindirs_with_mask(pio1, push_sm, 1 << sclk_pin, 1 << sclk_pin);
  pio_sm_set_pindirs_with_mask(pio1, push_sm, 1 << cs_pin, 1 << cs_pin);

  //----------------------------------------------------------------------------

  gpio_pull_up(miso_pin);
  gpio_set_input_hysteresis_enabled(miso_pin, false);

  for (uint32_t i = data0_pin; i <= clk_pin; ++i) {
    gpio_pull_up(i);
    gpio_set_input_hysteresis_enabled(i, false);
    pio_gpio_init(pio0, i);
  }

  for (uint32_t i = cs_pin; i <= sclk_pin; ++i) {
    pio_gpio_init(pio1, i);
  }
  for (uint32_t i = mosi0_pin; i <= mosi_last_pin; ++i) {
    pio_gpio_init(pio1, i);
  }

  for (uint32_t i = cs_pin; i <= mosi0_pin; ++i) {
    gpio_set_drive_strength(i, GPIO_DRIVE_STRENGTH_2MA);
  }

  pio_sm_init(pio0, pull_sm, ft1248_offset + ft1248_offset_entry_point,
              &pull_c);
  pio_sm_init(pio1, push_sm, pspi_offset + pspi_offset_entry_point, &push_c);
}

static void print_hex(char *out, uint32_t value) {
  for (int32_t i = 7; i >= 0; i--) {
    uint32_t digit = value & 0xF;
    out[i] = digit + ((digit < 10) ? '0' : ('A' - 10));
    value >>= 4;
  }
}

void core1_main(void) {
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

void core0_main(void) {
  if (CPU_FREQ_MHZ != 125) {
    for (uint32_t i = 0; i < 20 * 1000 * 1000; ++i) {
      NOP;
    }
  }

  uint32_t pull_sm = 0;
  uint32_t push_sm = 1;

  uint32_t iter = 0;
  uint32_t errors = 0;
  uint32_t bytes = 0;
  while (!multicore_fifo_rvalid()) {
    NOP;
  }

  pio_sm_set_enabled(pio0, pull_sm, true);
  pio_sm_set_enabled(pio1, push_sm, true);

  // Each item is 2 bits x 16 channel;
  // bundle 8 of them for word-oriented transfer.
#define BUNDLE_LEN 8

  // bytes per second: 22579200 = 44100*256*2
  // items per second: 5644800 = 22579200 / 4
  // bundles per second: 705600 = 5644800 / 8
  // bundles per 1000us: 705.6

  uint32_t bundles_step_int = 705;
  uint32_t bundles_step_rem = 6;
  uint32_t bundles_step_div = 10;
  uint32_t tick_step = 1000;

  uint32_t read_pos = 0;
  uint32_t read_pos_target = 0;
  uint32_t read_pos_tail = 0;
  uint32_t write_pos = 0;
  uint64_t next_tick = time_us_64() + tick_step;
  uint32_t lag = RING_BUFFER_SIZE - 4096;
#define MY_SPI 0
  uint32_t next_spi = 0;

  uint32_t num_restarts = 0;

  while (1) {
    // Pull just one.
    while ((write_pos < read_pos + lag) &&
           !pio_sm_is_rx_fifo_empty(pio0, pull_sm)) {
      uint32_t encoded = pio_sm_get(pio0, pull_sm);
      uint32_t decoded = encoded - ((encoded >> 7) & 0x01010101);
      ring_buffer[write_pos++ & RING_BUFFER_MASK] = decoded;
    }
    // Update target, if necessary.
    if (time_us_64() >= next_tick) {
      if (write_pos == 0) { // Still waiting for input.
        next_tick = time_us_64() + (tick_step / 2);
      } else {
        if (read_pos != read_pos_target) {
          // Ooops, previous transfer is incomplete
          // TODO: report problem
        }
        read_pos_target += bundles_step_int * BUNDLE_LEN;
        read_pos_tail += bundles_step_rem;
        if (read_pos_tail >= bundles_step_div) {
          read_pos_tail -= bundles_step_div;
          read_pos_target += BUNDLE_LEN;
        }
        next_tick += tick_step;
      }
    }

#if MY_SPI
#define PUSH                                                                   \
  pio_sm_put(pio1, push_sm, text[next_spi++ & 0x7]);                           \
  read_pos++;
#else
#define PUSH                                                                   \
  pio_sm_put(pio1, push_sm, ring_buffer[read_pos++ & RING_BUFFER_MASK]);
#endif
    // Push as many as possible.
    // errors = (errors << 4) | pio_sm_get_tx_fifo_level(pio1, push_sm);
    if ((read_pos < read_pos_target) &&
        (pio_sm_get_tx_fifo_level(pio1, push_sm) <= 3)) {
      PUSH;
      PUSH;
      PUSH;
      PUSH;
      PUSH;
      for (uint32_t i = 0; i < 3; ++i) {
        while (pio_sm_is_tx_fifo_full(pio1, push_sm)) {
          // no-op
        }
        PUSH;
      }
    }
    // If next read is after next write -> last read was at or after next write.
    if (read_pos > write_pos) {
      read_pos = 0;
      read_pos_tail = 0;
      write_pos = 0;
      // Give puller some advance.
      next_tick = time_us_64() + (tick_step / 2);
      read_pos_target = 0;
      num_restarts++;
    }
#if USE_DISPLAY
    if (multicore_fifo_rvalid()) {
      (void)multicore_fifo_pop_blocking();
      multicore_fifo_push_blocking(read_pos);
      multicore_fifo_push_blocking(read_pos_target - read_pos);
      // multicore_fifo_push_blocking(write_pos - read_pos);
      multicore_fifo_push_blocking(errors);
    }
#endif
  }
}

void core1_start(void) {
  uint32_t test = multicore_fifo_pop_blocking();
  if (test == kCore0Ready) {
    multicore_fifo_push_blocking(kCore1Ready);
    core1_main();
  }
  while (1) {
    tight_loop_contents();
  }
}

int main(void) {
  init_flash();

  // Make core0 the only high-priority master; this way is never stalled by
  // memory access arbitration.
  //                       CPU_0      CPU_1      DMA_R      DMA_W
  bus_ctrl_hw->priority = (1 << 0) | (0 << 4) | (0 << 8) | (0 << 12);
  while (!bus_ctrl_hw->priority_ack) {
    // no-op
  }

  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    // Wait until voltage is stable.
    flash();
    set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
    // Wait until clock is stable.
    flash();
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    CPU_FREQ_KHZ * 1000, CPU_FREQ_KHZ * 1000);
    flash();
  }

  // Deinits flash.
  init_pio();

#if USE_DISPLAY
  multicore_launch_core1(core1_start);
  multicore_fifo_push_blocking(kCore0Ready);
  uint32_t test = multicore_fifo_pop_blocking();
  if (test == kCore1Ready) {
    core0_main();
  }
#else
  core0_main();
#endif

  while (1) {
    tight_loop_contents();
  }
}
