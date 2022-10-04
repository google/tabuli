#include "hardware/address_mapped.h"
#include "sound.pio.h"

#include "hardware/gpio.h"
#include "hardware/structs/systick.h"
#include "hardware/vreg.h"
#include "pico/assert.h"
#include "pico/stdlib.h"

#include <hardware/pio.h>
#include <stdint.h>
#include <string.h>

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
  sm_config_set_out_shift(&c, /* shift_right */ true, /* autopull */ false,
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

    for (uint32_t i = 0; i < 2; ++i) {
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
#define DEBUG_CLK 0
#define CLK_PER_LOOP (896 + 80)

uint16_t samples[256 * 16 + 16];

static uint32_t precomputed[129 * 4] = {
    0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
    0x4000000,  0x0,        0x1000,     0x80000,    0x0,        0x0,
    0x0,        0x4A0000,   0x0,        0x0,        0x10,       0x40,
    0x0,        0x600,      0x10000000, 0x1000,     0x800088,   0x0,
    0x400000,   0x0,        0x21000002, 0x2400,     0x204,      0x1000002,
    0x0,        0x20010080, 0x10008,    0x0,        0x80,       0x800A0C,
    0x8,        0x20004042, 0x820000,   0x4100,     0x40,       0x10400,
    0x400003,   0x11200001, 0x100,      0x14,       0x41002000, 0xA000228,
    0xC050000B, 0xC01,      0x20000,    0x40,       0x82000000, 0x82000080,
    0x12100,    0x48041100, 0x21104100, 0x700100,   0x2000040,  0xC00002,
    0x80000000, 0x1812001,  0x42400,    0x80060620, 0x21000000, 0x40000020,
    0x50010124, 0x3440030,  0x8022880,  0x10010088, 0x4800A010, 0x50800000,
    0x18400010, 0x40C12000, 0x1102010,  0x45000021, 0x8404000,  0x24023040,
    0x60070100, 0x4001300,  0x100100,   0x54808081, 0x40000B00, 0x41802C40,
    0x80094800, 0x100000A,  0x882A4202, 0x8110240,  0x8A200618, 0x1100802,
    0x5044011,  0x220000A,  0x1710A010, 0x8000102,  0x441600,   0x812B004,
    0x5003230,  0x88000B00, 0x4A088A4,  0x10200A00, 0x13100909, 0x20421200,
    0x80110214, 0x80A884,   0x189A1495, 0x404108,   0x42204200, 0xA8000410,
    0x1A04C244, 0x4800880,  0x92804282, 0x8001909,  0x20040F81, 0x1306404,
    0x48024800, 0x9192240,  0xC88088A8, 0x2920010,  0x540B0020, 0x10390848,
    0x40000EA,  0xD1004102, 0x10050100, 0xA671784,  0x143D004A, 0x22811A04,
    0x2989C010, 0x282001,   0x14,       0x2801CC60, 0x71241303, 0x2844112B,
    0x17C0022A, 0x40585440, 0x10641004, 0x8BA0C80,  0x10002409, 0x80C041A,
    0x9A880264, 0x71840A59, 0x13090A4,  0x22A90805, 0xCA4D0440, 0xA5128400,
    0x80868080, 0x510815,   0x4757212B, 0x6880042C, 0x10206290, 0x9A9A4021,
    0x298C502,  0x689212C0, 0xA029118,  0x561F1488, 0x710008A8, 0xD284114,
    0x7031110C, 0x74360602, 0xA9600A0F, 0xA0044404, 0xD2101F34, 0x13490204,
    0x64400344, 0x18222473, 0x2025DA04, 0x69D8140,  0x5A3A7A22, 0x940A0040,
    0x2B0341,   0x303313C1, 0x87635,    0x40F2C88A, 0x15820043, 0x61C98882,
    0x14EF0085, 0x2CD0041F, 0x10C617D1, 0x28944498, 0x32C49112, 0x47282418,
    0xF63E500C, 0xE4130600, 0x30550030, 0x3B9451A,  0x834C080D, 0x1377530,
    0x1019C54B, 0x65980AA,  0xB9897E58, 0x9013009,  0x4501830C, 0xC36105A7,
    0x97101E20, 0x6309ED5E, 0x97708082, 0x9061C014, 0x2F14781B, 0xEA598CC4,
    0x7003C604, 0x90C1420C, 0xE1D2041,  0x7353D50A, 0x4A00DA10, 0x3445E879,
    0x17C980BA, 0x1C8A81AA, 0xB20C44C0, 0xE0C42E3D, 0x85476998, 0x35202D6,
    0x4D47082B, 0xF21A4929, 0x65702222, 0x2E8450E5, 0x96BB63C5, 0xE9650009,
    0xD0E10078, 0x2496562,  0xD9B80D6A, 0x9DE5107C, 0x329A0A16, 0x5C4831EA,
    0x27E1D090, 0xA919D971, 0x9702208E, 0x41D3CE68, 0xA26A65A6, 0xF8280D5E,
    0x9704A194, 0x53DDE022, 0x85A9B102, 0xC0D757B9, 0xEAAAC70B, 0x6206742C,
    0x27AB21B0, 0x5CA615C3, 0xFE306537, 0x280109CA, 0x8B5B6494, 0x92EB7513,
    0x600850F,  0x8CCA06DE, 0x3FAECE9B, 0xAA296C3A, 0xBA0C8088, 0xABD7F968,
    0xBA97B243, 0x3B895461, 0xD280696E, 0x220309D7, 0xBE57D41E, 0x9ABBB431,
    0xEC0A6B01, 0xFA7A4547, 0xB64C14DE, 0x5E7D4152, 0xA70EBADB, 0xAB32D226,
    0xED917936, 0x96A001C7, 0x788B97B9, 0x99DF9E66, 0x9BA82584, 0xD256840F,
    0xCACDDA89, 0x3C006D37, 0xF912D9A9, 0x5F1D6C47, 0x5F785505, 0x9347C6D7,
    0x595AFF46, 0xF0A1C58C, 0xBFED0343, 0x7A14735F, 0x624BAFF3, 0xA501592D,
    0x902AE798, 0x4A8A1866, 0x59B5EFF8, 0x5CFEB67B, 0xD73A7E8C, 0x7EDDF54F,
    0x5804E610, 0x9AF2AD6C, 0x917F7EC4, 0xB94FE027, 0x13F7586F, 0x2749AF64,
    0xE47B0747, 0xEB7B1713, 0x82EE63FD, 0xD60ED3AA, 0x3DFFE65C, 0x38074C72,
    0xACDAB97D, 0xB03EC9F9, 0xB9153A73, 0xD7DCEE0F, 0x668F3753, 0xE1797273,
    0x2FF1FB1D, 0x3C96C6D7, 0x121AFEE2, 0x5F7CB768, 0x68516E7F, 0xE6CCB71A,
    0xDDAFF9FF, 0x783E896,  0x34B4106F, 0xB767A79E, 0x36475C7B, 0x7FE63FBB,
    0xDBDDAEF5, 0x643D46B8, 0x55DDFDEF, 0x88563CBE, 0xBEC34B79, 0xC4F5FEEF,
    0xAE973B7,  0x63BFA587, 0x5977B07B, 0xC78D7FD2, 0xFE6D85B5, 0xEA9E9BAF,
    0x97F84D2A, 0xDF5ABD7C, 0xBDE9057E, 0xEFF79F1A, 0x7D7D3F77, 0x46676F63,
    0xD1BFFA83, 0xFCEF8E98, 0x8CF7DAC7, 0xC3DFBB3E, 0x467ECBAF, 0x9B1DF9FC,
    0x737D25B7, 0xF7A2F965, 0xC8BFE5AF, 0xAFBE9F7C, 0xE7F760BB, 0x8FF7ADF2,
    0xC6DDF97A, 0xB2F32DF7, 0xEFE7DFED, 0x4EF5EF5E, 0x2D46D1EF, 0xFCF713E8,
    0x75D557FB, 0x74E91E7B, 0xDB5BFFF,  0xFB4BBEF6, 0x26AEFFA4, 0x1FDFEFFF,
    0xFB47DBEF, 0x29DBA4EB, 0xFF27FE6F, 0x3BFFDEC7, 0xF567B5C5, 0x5E5AB65F,
    0xDEFA9F7F, 0x75AEEF4D, 0xBBF363D7, 0x9579CEFF, 0x75F0BAF9, 0xFEE7CAEE,
    0x6FFEEF07, 0xAFFFF7B1, 0x7FFEEAEF, 0xEFD79A6E, 0xD67D635F, 0x7B3E3EBF,
    0xBE9FCBFF, 0xAAFDEBD3, 0xEEEE79BF, 0xD7FD5CF4, 0xF2FF8FBF, 0xBAF3FB3B,
    0x6BFFE7E7, 0x2E777A7E, 0xB7EAFFFF, 0xCFE3DFDF, 0xAFEBB7D6, 0x7BBEA8E6,
    0x2EBD779F, 0xDEF6BEF3, 0x7C1DDFF5, 0xFFEFFF75, 0xFFDBD7FF, 0xDFFDCF7B,
    0x7BBFF79C, 0x68EB71E7, 0xD9BA2FFB, 0xEFFD4DF9, 0xF7AFF2EF, 0xDFB77F7F,
    0xF3FFFE7F, 0x4458BFBF, 0x3FFFBFBF, 0xF3C5FDDF, 0x7F5DDFFD, 0xFF3DFFBE,
    0x77F1DFBF, 0x1DFF5B5B, 0xEFAFBFED, 0xFD7ECB71, 0xECB9DFDF, 0x7FFFFDFE,
    0xFFFF7AB7, 0xADDF2FFC, 0xFF6FFBEE, 0xFFF5E6DD, 0xDD9F7EEF, 0xFBDDFB3B,
    0xFBFDFFAF, 0xBFDDF7F4, 0xE57FE7BF, 0xDBEFE7FB, 0xF5EB7CEF, 0x9FFFFF7F,
    0xDDDDEDFF, 0xFD773DEF, 0xFFFFD5FF, 0x57FEBFF3, 0xFDF9F7BF, 0xBFBEEDFF,
    0xDFE7BB5B, 0x7FF7BF7F, 0xFE37D6FF, 0x3FB7EEFF, 0xFDFFFF97, 0xFFFEFFE6,
    0xB5FFBFBF, 0xF9FEDB6F, 0xFFFFBFFF, 0xADFFFBB5, 0xFFBAFB7B, 0xFA77FFFA,
    0xEBFDFFFF, 0xFBFBDF7F, 0xFEBBFEEE, 0xFFFFBFD7, 0xEEFFFDF2, 0xFFFF7EBF,
    0xDDFFBFFF, 0xBDFDFFFF, 0xFFBBFDFA, 0xFB9EEFDF, 0xFBFF7DFC, 0xFFFF5EFE,
    0xDDD7BFFF, 0x7FFFDFFF, 0xFFBDFFBB, 0xEFF7FFFE, 0xC7FDBFFF, 0xFF7FAFFF,
    0xFFFFFBFE, 0xDFFE7DDF, 0xFFF7FFBB, 0xAEFFFFBF, 0xFEFFDDFF, 0xEBFFFFDD,
    0xFFBBFEED, 0xFFFFDFFF, 0xFFFFFFE7, 0xFAFFFEF7, 0xFFBBFFFF, 0xFFF7FDFC,
    0xB7FEFEFF, 0xFFFFFFFE, 0x7FFFDFBF, 0xDFFFBBFF, 0xB7FFFFFF, 0xEFDE3FFF,
    0xFDFFFCFF, 0xFFFFFFFF, 0xFCFFFFFF, 0xFFFFF7FF, 0xFE7FFFEF, 0xFEFFF77F,
    0xFFFFFFFB, 0xFFFF37FF, 0x7FFEFFBF, 0xFFFFEFFF, 0xFFFFDFFF, 0xFFBFFFFD,
    0xBFFFFFFF, 0xFFBFF7FE, 0xFFDFBFFF, 0xFFFFFFFF, 0xFF7FFDFF, 0xFFF6FFFF,
    0xFFFFFFEF, 0xD7AFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFDFFFFF, 0xF7FFFFFF,
    0xFFF7FDFF, 0xFFFFFFFF, 0xF7FFBFFF, 0xFFFFFFFF, 0xFFFFFFDF, 0xFFFFFFFF,
    0xFDFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFDFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFBF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

uint16_t sinw[256] = {
    32768, 33572, 34375, 35178, 35979, 36779, 37576, 38370, 39160, 39947, 40729,
    41507, 42280, 43046, 43807, 44561, 45307, 46046, 46778, 47500, 48214, 48919,
    49614, 50298, 50972, 51636, 52287, 52927, 53555, 54171, 54773, 55362, 55938,
    56500, 57047, 57580, 58098, 58600, 59087, 59558, 60013, 60452, 60874, 61279,
    61666, 62037, 62389, 62724, 63041, 63340, 63620, 63882, 64125, 64349, 64553,
    64739, 64906, 65053, 65181, 65289, 65378, 65447, 65496, 65526, 65535, 65526,
    65496, 65447, 65378, 65289, 65181, 65053, 64906, 64739, 64553, 64349, 64125,
    63882, 63620, 63340, 63041, 62724, 62389, 62037, 61666, 61279, 60874, 60452,
    60013, 59558, 59087, 58600, 58098, 57580, 57047, 56500, 55938, 55362, 54773,
    54171, 53555, 52927, 52287, 51636, 50972, 50298, 49614, 48919, 48214, 47500,
    46778, 46046, 45307, 44561, 43807, 43046, 42280, 41507, 40729, 39947, 39160,
    38370, 37576, 36779, 35979, 35178, 34375, 33572, 32768, 31963, 31160, 30357,
    29556, 28756, 27959, 27165, 26375, 25588, 24806, 24028, 23255, 22489, 21728,
    20974, 20228, 19489, 18757, 18035, 17321, 16616, 15921, 15237, 14563, 13899,
    13248, 12608, 11980, 11364, 10762, 10173, 9597,  9035,  8488,  7955,  7437,
    6935,  6448,  5977,  5522,  5083,  4661,  4256,  3869,  3498,  3146,  2811,
    2494,  2195,  1915,  1653,  1410,  1186,  982,   796,   629,   482,   354,
    246,   157,   88,    39,    9,     0,     9,     39,    88,    157,   246,
    354,   482,   629,   796,   982,   1186,  1410,  1653,  1915,  2195,  2494,
    2811,  3146,  3498,  3869,  4256,  4661,  5083,  5522,  5977,  6448,  6935,
    7437,  7955,  8488,  9035,  9597,  10173, 10762, 11364, 11980, 12608, 13248,
    13899, 14563, 15237, 15921, 16616, 17321, 18035, 18757, 19489, 20228, 20974,
    21728, 22489, 23255, 24028, 24806, 25588, 26375, 27165, 27959, 28756, 29556,
    30357, 31160, 31963};

void __attribute__((noinline))
merge(uint32_t *to, uint32_t *from1, uint32_t *from2) {
  for (uint32_t j = 0; j < 8; ++j) {
    to[j * 8] = from1[j] | from2[j];
  }
}

typedef struct Cookie {
  uint16_t bank[16];
  uint16_t qs[16];
  uint32_t sample_pos;
  uint32_t sample_tail;
} Cookie;

void main_loop(Cookie cookie, uint16_t *sample_base);

int main() {
  if (CPU_FREQ_MHZ != 125) {
    vreg_set_voltage(VREG_VOLTAGE_1_30);
    set_sys_clock_khz(CPU_FREQ_MHZ * 1000, true);
    // Wait until clock is stable.
    for (uint32_t i = 0; i < 5000000; ++i) {
      NOP;
    }
  }

  for (uint32_t i = 0; i < 256; ++i) {
    for (uint32_t j = 0; j < 16; ++j) {
      samples[i * 16 + j] = sinw[(i * (j + 1)) & 0xFF];
    }
  }
  for (uint32_t j = 0; j < 16; ++j) {
    samples[256 * 16 + j] = samples[j];
  }

  gpio_init(16);
  gpio_set_dir(16, GPIO_OUT);

  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);

  // gpio_set_slew_rate(x, GPIO_SLEW_RATE_FAST);
  // gpio_set_drive_strength(x, GPIO_DRIVE_STRENGTH_2MA);

  sound_program_init(0);
  sound_program_init(1);

  // Prefill TX FIFO.
  for (uint32_t i = 0; i < 8; ++i) {
    for (uint32_t j = 0; j < 4; ++j) {
      pio0->txf[j] = 0;
      pio1->txf[j] = 0;
    }
  }

  Cookie cookie;
  memset(&cookie, 0, sizeof(cookie));

  main_loop(cookie, samples);
}

void __attribute__((noinline)) dummy(Cookie *cookie) {
  asm volatile("" : : : "memory");
}

void main_loop(Cookie cookie, uint16_t *sample_base) {
  io_wo_32 *set = &sio_hw->gpio_set;
  io_wo_32 *clr = &sio_hw->gpio_clr;
  io_wo_32 *togl = &sio_hw->gpio_togl;
  // uint32_t dbg[16] = {0};
#if DEBUG_CLK
  systick_hw->rvr = MAX_TICK;
  systick_hw->csr = 5; // enable + no-int + cpu-clk
#endif

  // pio_set_sm_mask_enabled(pio0, 0xF, true);
  // pio_set_sm_mask_enabled(pio1, 0xF, true);
  // --
  // pio0->ctrl = 0xF;
  // pio1->ctrl = 0xF;
  // --
  io_wo_32 *pio0_ctrl = &pio0->ctrl;
  io_wo_32 *pio1_ctrl = &pio1->ctrl;
  uint32_t all_sms = 0xF;
  // Force PIO SM start with predictable delay (2 ticks)
  asm volatile(".thumb\n\t"
               ".syntax unified\n\t"
               "str %2, [%0, 0]\n\t"
               "str %2, [%1, 0]\n\t"
               : /* no output */
               : "l"(pio0_ctrl), "l"(pio1_ctrl), "l"(all_sms)
               : "memory");

  // After launching SMs immediately consume 2 items; replenish.
  pio0->txf[0] = 0;
  pio1->txf[0] = 0;
  pio0->txf[1] = 0;
  pio1->txf[1] = 0;
  pio0->txf[2] = 0;
  pio1->txf[2] = 0;
  pio0->txf[3] = 0;
  pio1->txf[3] = 0;
  pio0->txf[0] = 0;
  pio1->txf[0] = 0;
  pio0->txf[1] = 0;
  pio1->txf[1] = 0;
  pio0->txf[2] = 0;
  pio1->txf[2] = 0;
  pio0->txf[3] = 0;
  pio1->txf[3] = 0;

  while (true) {
    cookie.sample_pos += kSampleStepInt;
    cookie.sample_tail += kSampleStepRem;
    if (cookie.sample_tail >= kSampleStepDiv) {
      cookie.sample_tail -= kSampleStepDiv;
      cookie.sample_pos++;
    }

    uint32_t next_mul = cookie.sample_pos & 0xFFFF;
    uint32_t mul = 0x10000 - next_mul;

    // 1 << 16 is sample step multiplier
    // 1 << 4 stands for number of channels
    // Each scratch memory bank is 4096 bytes = 128 * (2 * 16)
    // Scratch banks are mapped without a gap, giving 256 samples in total.
    // TODO: remember about tail copying!
    uint32_t sample_idx = (cookie.sample_pos >> 16) & 0xFF;
    uint16_t *sample = sample_base + sample_idx * 16;

#pragma GCC unroll 16
    for (uint32_t i = 0; i < 16; ++i) {
      uint32_t tmp1 = sample[i]; // fixed offset after unroll
      tmp1 *= mul;
      uint32_t tmp2 = sample[i + 16]; // fixed offset after unroll
      tmp2 *= next_mul;
      tmp1 += tmp2;
      tmp1 >>= 16;
      tmp2 = cookie.qs[i]; // on stack

      // tmp1 += tmp2;
      //  Alas, GCC is too buggy to do that!
      asm(".thumb\n\t"
          ".syntax unified\n\t"
          "adds %0, %1, %2"
          : "=l"(tmp1)
          : "0"(tmp1), "l"(tmp2));

      tmp2 = tmp1 << (32 - 9);
      tmp2 >>= (32 - 9);
      cookie.qs[i] = tmp2; // on stack

      tmp1 >>= 9;
      tmp1 <<= 2;
      cookie.bank[i] = tmp1; // on stack
    }

    io_wo_32 *pio0txf = pio0->txf;
    io_wo_32 *pio1txf = pio1->txf;
#pragma GCC unroll 4
    for (uint32_t j = 0; j < 4; ++j) {
      while (pio_sm_is_tx_fifo_full(pio0, 0)) {
        // wait for FIFO; all SM should become ready since they are synced
      }
      const uint32_t *base = precomputed + j;
      pio0txf[0] = base[cookie.bank[0]];
      pio0txf[0] = base[cookie.bank[1]];
      pio0txf[1] = base[cookie.bank[2]];
      pio0txf[1] = base[cookie.bank[3]];
      pio0txf[2] = base[cookie.bank[4]];
      pio0txf[2] = base[cookie.bank[5]];
      pio0txf[3] = base[cookie.bank[6]];
      pio0txf[3] = base[cookie.bank[7]];
      pio1txf[0] = base[cookie.bank[8]];
      pio1txf[0] = base[cookie.bank[9]];
      pio1txf[1] = base[cookie.bank[10]];
      pio1txf[1] = base[cookie.bank[11]];
      pio1txf[2] = base[cookie.bank[12]];
      pio1txf[2] = base[cookie.bank[13]];
      pio1txf[3] = base[cookie.bank[14]];
      pio1txf[3] = base[cookie.bank[15]];
      // dbg[j] = pio0->fdebug | pio1->fdebug;
    }

#if DEBUG_CLK
    uint32_t tick = systick_hw->cvr;
    uint32_t delta = MAX_TICK - tick;
    uint32_t in_time = ((delta - CLK_PER_LOOP) >> 31) & 1;
    *(in_time ? set : clr) = 1 << 25;
    systick_hw->cvr = 0;
#endif
    uint32_t dbg = pio0->fdebug | pio1->fdebug;
    *((dbg & 0xFF000000) ? set : clr) = 1 << 16;
    *((dbg & 0x00FF0000) ? set : clr) = 1 << 25;
    // pio0->fdebug = 0xFFFF0000;
    // pio1->fdebug = 0xFFFF0000;
  }
}
