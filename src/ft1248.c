#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "pico/stdlib.h"
#include <stdint.h>

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420

int main() {
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
  // 0x0FF - data[0..7]
  // 0x0100 - sclk
  // 0x0200 - ss_n
  // 0x0400 - miso
  // 0x0800 - status1
  // 0x1000 - status2
  // 0x2000 - status3

  gpio_init_mask(0x3FFF); // 14 bits - 8 data, sclk, ss_n, miso, 3 status
  // by default all are input, no value (0)

  gpio_set_mask(0x200);           // Turn ss_n to high before making it output
  gpio_set_dir_out_masked(0x3B00); // sclk, ss_n, miso, status

  NOP;
  NOP;
  NOP;

  // Read command is 0x1;
  // data pins: (LSB) cmd3 X X cmd2 0 cmd1 cmd0 X;
  // if X is 0, then only 6-th bit is set (cmd0)
  gpio_clr_mask(0xFF);
  gpio_set_mask(0x40);

  uint8_t expected = 0;

  while (1) { // Read in infinite loop.
    gpio_set_mask(0x800); // status cmd on
    // Send "read" command first.
    gpio_clr_mask(0x200); // "Select" slave.
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;

    gpio_set_mask(0x100); // before that we are not allowed to drive data pins.
    NOP;
    NOP;
    NOP;
    gpio_set_dir_out_masked(0xFF); // data out pins are already configured
    NOP;
    NOP;

    gpio_clr_mask(0x100);
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;

    // First cycle is over.
    // Now "bus turnaround" happens.
    gpio_set_mask(0x100);
    NOP;
    NOP;
    gpio_set_dir_in_masked(0xFF);
    uint32_t rxf = gpio_get_all() & 0x400;
    gpio_clr_mask(0x800); // status cmd off
    if (rxf) { // nothing to read; restore status-quo
      gpio_clr_mask(0x100);
      NOP;
      NOP;
      NOP;
      gpio_set_mask(0x200);
      NOP;
      NOP;
      NOP;
      continue;
    }

    gpio_set_mask(0x1000); // status read on
    gpio_clr_mask(0x100);
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    // Now we can read until NAK
    while (1) {
      gpio_set_mask(0x100);
      NOP;
      NOP;
      uint32_t pins = gpio_get_all();

      gpio_clr_mask(0x100);
      NOP;
      NOP;
      uint8_t data = pins;
      uint32_t nak = pins & 0x400;
      if (nak) {
        // Read aborted
        break;
      }
      if (data != expected) {
        gpio_set_mask(0x2000); // status error on
      } else {
        gpio_clr_mask(0x2000); // status error off
      }
      expected = data + 1;
    }
    // Restore status-quo
    gpio_clr_mask(0x1000); // status read off
    gpio_clr_mask(0x2000); // status error off
    gpio_set_mask(0x200);
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
    NOP;
  }
}
