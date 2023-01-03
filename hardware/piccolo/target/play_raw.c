#include "play.h"

#include "hardware/pio.h"

volatile uint32_t read_pos = 0;
volatile uint32_t write_pos = 0;
volatile uint32_t rw_flag = 0;

void play(Cookie cookie, uint16_t *src) {
  pio_set_sm_mask_enabled(pio0, 0xF, true);

  // After launching SMs immediately consumes; replenish.
  pio0->txf[0] = 0;
  pio0->txf[1] = 0;
  pio0->txf[2] = 0;
  pio0->txf[3] = 0;

  while (rw_flag == 1) {
    uint32_t increment = 1;
    uint32_t pos = read_pos + increment * 16;
    read_pos = pos;
    uint16_t *sample_addr = src + (pos & BUF_MASK);
    uint32_t *sample = (uint32_t *)sample_addr;

    io_wo_32 *pio0txf = pio0->txf;

    for (uint32_t j = 0; j < 2; ++j) {
      while (pio_sm_is_tx_fifo_full(pio0, 0)) {
        // wait for FIFO; all SM should become ready since they are synced
      }
      pio0txf[0] = sample[j + 0];
      pio0txf[1] = sample[j + 4];
      pio0txf[2] = sample[j + 8];
      pio0txf[3] = sample[j + 12];
    }
  }
}
