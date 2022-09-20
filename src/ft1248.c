#include "display.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include <stdint.h>
#include <stdio.h>

#define NOP asm volatile("nop");

#define CPU_FREQ_MHZ 420

void core1_main() {
  display_init();
  display_print(0, 0, "DEAD");
  display_print(4, 6, "BEEF");
  display_print(8, 12, "CAFE");
  display_print(12, 18, "BABE");
  display_flush();

  char txt[64];

  uint32_t next_line = 0;
  uint32_t index = 0;
  multicore_fifo_push_blocking(0); // signal we are ready
  while (1) {
    index++;
    uint32_t iter = multicore_fifo_pop_blocking();
    uint32_t errors = multicore_fifo_pop_blocking();
    uint32_t bytes = multicore_fifo_pop_blocking();
    multicore_fifo_push_blocking(0); // signal we are ready
    sprintf(txt, "%08X%c%08X/%08X     ", iter, index & 1 ? ':' : ' ', errors,
            bytes);
    display_print(0, next_line * 6, txt);
    display_flush();
    next_line++;
    if (next_line == 9) {
      next_line = 0;
    }
  }
}

void core0_main() {
  uint32_t iter = 0;
  uint32_t errors = 0;
  uint32_t bytes = 0;
  while (!multicore_fifo_rvalid()) {
    NOP;
  }

#define DATA 0x0FF
#define MISO 0x100
#define SS 0x200
#define CLK 0x400
#define HI(S) gpio_set_mask(S)
#define LO(S) gpio_clr_mask(S)

  gpio_init_mask(DATA | CLK | SS | MISO);
  // by default all are input, no value (0)

  HI(SS);
  LO(CLK);
  gpio_set_dir_out_masked(CLK | SS);
  // Read command is 0x1;
  // data pins: (LSB) cmd3 X X cmd2 0 cmd1 cmd0 X;
  // if X is 0, then only 6-th bit is set (cmd0)
  gpio_clr_mask(DATA);
  gpio_set_mask(0x40);

  for (uint32_t i = 0; i < 20 * 1000 * 1000; ++i) {
    NOP;
  }

  uint32_t rxf = 0;
  uint32_t volatile pins = 0;

send_read_command: // Read in infinite loop.
  if (multicore_fifo_rvalid()) {
    (void)multicore_fifo_pop_blocking();
    multicore_fifo_push_blocking(iter);
    multicore_fifo_push_blocking(errors);
    multicore_fifo_push_blocking(bytes);
  }

  // Send "read" command first.
  LO(SS); // "Select" slave.
  NOP;
  NOP;

  HI(CLK); // before that we are not allowed to drive data pins.
  NOP;
  NOP;

  // data out pins are already configured, just switch direction
  gpio_set_dir_out_masked(DATA);
  NOP;
  NOP;

  LO(CLK);
  NOP;
  NOP;

  // First cycle is over.
  // Now "bus turnaround" happens.
  HI(CLK);
  gpio_set_dir_in_masked(DATA);
  NOP;
  NOP;

  LO(CLK);
  pins = gpio_get_all();
  rxf = pins & MISO;
  NOP;
  NOP;
  if (rxf) { // nothing to read; restore status-quo
    goto release_line;
  }
  iter++;

  // Now we can read until NAK
  HI(CLK);
  NOP;
  NOP;
  NOP;
read_byte:
  LO(CLK);
  NOP;
  NOP;
  NOP;
  NOP;
  pins = gpio_get_all();
  uint32_t nak = pins & MISO;
  if (nak) {
    goto release_line;
  }
  HI(CLK);
  bytes++;
  NOP;
  goto read_byte;

  // Restore status-quo
release_line:
  LO(CLK);
  NOP;
  NOP;

  HI(SS);
  NOP;
  NOP;

  goto send_read_command;
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
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
  // Wait until clock is stable.
  for (uint32_t i = 0; i < 50000; ++i) {
    NOP;
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