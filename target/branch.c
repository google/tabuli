#include <stdint.h>
#include <string.h>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/spi.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "play.h"
#include "sound.pio.h"
#include "sound_sd.pio.h"

#define CPU_FREQ_MHZ 420
#define CPU_FREQ_KHZ (CPU_FREQ_MHZ * 1000)

uint32_t kCore0Ready = 0xFEEDBAC0;
uint32_t kCore1Ready = 0xFEEDBAC1;
uint32_t kStartPlay = 0xC0DEABBA;

// First 4 pins are used for SPI.
#define AUDIO_PIN_0 4
// Use the other side for debug
#define AUDIO_PIN_LAST 15
// Free pin start: 20 = 4 + 16

// TODO: remember about tail copying!
uint16_t buffer[BUF_LEN + 16];

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

// Loads the program. SMs are (re-)configured by `prepare_pio`.
void init_pio() {
  pio_clear_instruction_memory(pio0);
  pio_clear_instruction_memory(pio1);
  if (PICCOLO_PLAY_RAW) {
    pio_add_program_at_offset(pio0, &sound_sd_program, 0);
  } else {
    pio_add_program_at_offset(pio0, &sound_program, 0);
    pio_add_program_at_offset(pio1, &sound_program, 0);
  }
}

// Resets SMs.
void prepare_pio() {
  pio_sm_config c = pio_get_default_sm_config();
  if (PICCOLO_PLAY_RAW) {
    // sm_config_set_in_pins(&c, data0_pin);
    // sm_config_set_sideset(&c, 0, false, false);
    sm_config_set_clkdiv_int_frac(&c, /* div_int */ 149, /* div_frac */ 0);
    sm_config_set_wrap(&c, sound_sd_wrap_target, sound_sd_wrap);
    // sm_config_set_jmp_pin(&c, miso_pin);
    sm_config_set_in_shift(&c, /* shift_right */ true, /* autopush */ false,
                           /* push threshold */ 32);
    sm_config_set_out_shift(&c, /* shift_right */ true, /* autopull */ true,
                            /* pull_threshold */ 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    // sm_config_set_out_special(&c, sticky, has_enable_pin, enable_pin_index);
    // sm_config_set_mov_status(&c, status_sel, status_n);

    for (uint32_t sm = 0; sm < 4; ++sm) {
      uint32_t out_pins = AUDIO_PIN_0 + sm;
      sm_config_set_out_pins(&c, out_pins, 1);
      // Clear pins.
      pio_sm_set_pins_with_mask(pio0, sm, 0, 1 << out_pins);
      // Set direction
      pio_sm_set_consecutive_pindirs(pio0, sm, out_pins, 1, true);

      pio_gpio_init(pio0, out_pins);
      gpio_set_drive_strength(out_pins, GPIO_DRIVE_STRENGTH_12MA);

      pio_sm_init(pio0, sm, sound_sd_offset_entry_point, &c);
    }
  } else {
    for (uint32_t pio_n = 0; pio_n < 2; ++pio_n) {
      PIO pio = (pio_n == 0) ? pio0 : pio1;
      uint32_t entry_point =
          (pio_n == 0) ? sound_offset_entry_point0 : sound_offset_entry_point1;

      // sm_config_set_in_pins(&c, data0_pin);
      sm_config_set_sideset(&c, 2, true, false); // 2 = 1 bit + enable
      sm_config_set_clkdiv_int_frac(&c, /* div_int */ 1, /* div_frac */ 0);
      sm_config_set_wrap(&c, sound_wrap_target, sound_wrap);
      // sm_config_set_jmp_pin(&c, miso_pin);
      sm_config_set_in_shift(&c, /* shift_right */ true, /* autopush */ false,
                             /* push threshold */ 32);
      sm_config_set_out_shift(&c, /* shift_right */ true, /* autopull */ false,
                              /* pull_threshold */ 32);
      sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
      // sm_config_set_out_special(&c, sticky, has_enable_pin,
      // enable_pin_index); sm_config_set_mov_status(&c, status_sel, status_n);

      for (uint32_t sm = 0; sm < 4; ++sm) {
        uint32_t out_pins = AUDIO_PIN_0 + pio_n * 8 + 2 * sm;
        sm_config_set_out_pins(&c, out_pins, 1);
        sm_config_set_sideset_pins(&c, out_pins + 1);
        // Clear pins.
        pio_sm_set_pins_with_mask(pio, sm, 0, 3 << out_pins);
        // Set direction
        pio_sm_set_consecutive_pindirs(pio, sm, out_pins, 2, true);

        for (uint32_t i = 0; i < 2; ++i) {
          pio_gpio_init(pio, out_pins + i);
          gpio_set_drive_strength(out_pins + i, GPIO_DRIVE_STRENGTH_12MA);
        }

        pio_sm_init(pio, sm, entry_point, &c);
      }
    }
  }
}

void core0_main(void) {
  Cookie cookie;

  while (1) {
    // Prepare things, so we could start immediately.
    memset(&cookie, 0, sizeof(cookie));

    prepare_pio();

    // Prefill TX FIFO.
    for (uint32_t i = 0; i < 8; ++i) {
      for (uint32_t j = 0; j < 4; ++j) {
        pio0->txf[j] = 0;
        pio1->txf[j] = 0;
      }
    }

    // Wait until we are ready to play.
    (void)multicore_fifo_pop_blocking();
    play(cookie, buffer);

    if (rw_flag == 2) {
      rw_flag = 3;
    }

    // Stop SMs.
    pio_set_sm_mask_enabled(pio0, 0xF, false);
    if (!PICCOLO_PLAY_RAW) {
      pio_set_sm_mask_enabled(pio1, 0xF, false);
    }
    // TODO: set pins to 0.
  }
}

void core1_main(void) {
  spi_hw_t *spi_hw = spi_get_hw(spi0);
  // Disable output.
  hw_set_bits(&spi_hw->cr1, SPI_SSPCR1_SOD_BITS);
  // But stuff the FIFO.
  for (uint32_t i = 0; i < 8; ++i) {
    spi_hw->dr = 0;
  }

  while (1) {
    for (uint32_t i = 0; i < LAG; ++i) {
      buffer[i] = 0;
    }
    uint32_t write_pos_wrap = BUF_LEN + 16;
    read_pos = 0;
    write_pos = LAG;
    rw_flag = 1;

    // Wait for transmission.
    while (!(spi_hw->sr & SPI_SSPSR_RNE_BITS)) {
      // no-op
    }
    multicore_fifo_push_blocking(kStartPlay);
    gpio_put(16, 1);

    uint32_t cntr = 0;
    // While enough input...
    while (read_pos < write_pos) {
      gpio_put(17, cntr++ < 64);
      while (spi_hw->sr & SPI_SSPSR_RNE_BITS) {
        cntr = 0;
        buffer[write_pos++ & BUF_MASK] = spi_hw->dr;
      }
      if (write_pos >= write_pos_wrap) {
        write_pos_wrap += BUF_LEN;
        memcpy(buffer + BUF_LEN, buffer, 16 * sizeof(buffer[0]));
      }
      gpio_put(18, read_pos + BUF_LEN > write_pos);
    }
    gpio_put(16, 0);
    gpio_put(17, 0);
    gpio_put(18, 0);
    // Request stop.
    rw_flag = 2;
    // Wait for stop.
    while (rw_flag != 3) {
      // no-op
    }
    // Status quo.
    rw_flag = 0;
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

  for (uint32_t i = 0; i < 3; ++i) {
    gpio_init(16 + i);
    gpio_set_dir(16 + i, GPIO_OUT);
  }

  init_pio();

  spi_init(spi0, CPU_FREQ_MHZ / 14);
  spi_set_slave(spi0, true);
  gpio_set_function(0, GPIO_FUNC_SPI); // RX
  gpio_set_function(1, GPIO_FUNC_SPI); // CSn
  gpio_set_function(2, GPIO_FUNC_SPI); // SCK
  gpio_set_function(3, GPIO_FUNC_SPI); // TX

  // It is not possible to have LSB_FIRST
  spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    // Wait until voltage is stable.
    flash();
    set_sys_clock_khz(CPU_FREQ_KHZ, true);
    // Wait until clock is stable.
    flash();
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    CPU_FREQ_KHZ * 1000, CPU_FREQ_KHZ * 1000);
    flash();
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
