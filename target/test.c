#include "display.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/structs/spi.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include <stdint.h>

#define USE_DISPLAY 1

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420
#define CPU_FREQ_KHZ (CPU_FREQ_MHZ * 1000)

static void print_hex(char *out, uint32_t value) {
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

  uint32_t iter = 0xFFFFFFFF;
  uint32_t errors = 0;
  uint32_t bytes = 0;
  while (!multicore_fifo_rvalid()) {
    NOP;
  }

  spi_init(spi0, CPU_FREQ_MHZ / 2);
  spi_set_slave(spi0, true);
  gpio_set_function(0, GPIO_FUNC_SPI); // RX
  gpio_set_function(1, GPIO_FUNC_SPI); // CSn
  gpio_set_function(2, GPIO_FUNC_SPI); // SCK
  gpio_set_function(3, GPIO_FUNC_SPI); // TX

  // It is not possible to have LSB_FIRST
  spi_set_format(spi0, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);

  spi_hw_t *spi_hw = spi_get_hw(spi0);
  while (1) {
    uint32_t spi_status = spi_hw->sr;
    while (spi_status & (SPI_SSPSR_TNF_BITS | SPI_SSPSR_RNE_BITS)) {
      if (spi_status & SPI_SSPSR_TNF_BITS) {
        spi_hw->dr = 0;
        spi_status = spi_hw->sr;
        continue;
      }

      if (spi_status & SPI_SSPSR_RNE_BITS) {
        //*dst++ = (uint8_t) spi_get_hw(spi)->dr;
        uint32_t val = spi_hw->dr;
        if (val != 0xCAFE) {
          iter = val;
          errors++;
        }
        bytes++;
        spi_status = spi_hw->sr;
        continue;
      }
      spi_status = spi_hw->sr;
    }

    //iter++;
#if USE_DISPLAY
    if (multicore_fifo_rvalid()) {
      (void)multicore_fifo_pop_blocking();
      //errors++;
      multicore_fifo_push_blocking(iter);
      multicore_fifo_push_blocking(errors);
      multicore_fifo_push_blocking(bytes);
    }
#endif
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
  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    // Wait until voltage is stable.
    for (uint32_t i = 0; i < 5000000; ++i) {
      NOP;
    }
    set_sys_clock_khz(CPU_FREQ_KHZ, true);
    // Wait until clock is stable.
    for (uint32_t i = 0; i < 5000000; ++i) {
      NOP;
    }
    clock_configure(clk_peri, 0,
                    CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS,
                    CPU_FREQ_KHZ * 1000, CPU_FREQ_KHZ * 1000);
  }

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
