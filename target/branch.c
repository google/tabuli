#include "hardware/address_mapped.h"
#include "sound.pio.h"

#include "hardware/gpio.h"
#include "hardware/spi.h"
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
  sm_config_set_sideset(&c, 2, true, false); // 2 = 1 bit + enable
  sm_config_set_clkdiv_int_frac(&c, /* div_int */ 1, /* div_frac */ 0);
  sm_config_set_wrap(&c, sound_wrap_target, sound_wrap);
  // sm_config_set_jmp_pin(&c, miso_pin);
  sm_config_set_in_shift(&c, /* shift_right */ true, /* autopush */ false,
                         /* push threshold */ 32);
  sm_config_set_out_shift(&c, /* shift_right */ true, /* autopull */ false,
                          /* pull_threshold */ 32);
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
  // sm_config_set_out_special(&c, sticky, has_enable_pin, enable_pin_index);
  // sm_config_set_mov_status(&c, status_sel, status_n);

  for (uint32_t sm = 0; sm < 4; ++sm) {
    uint32_t out_pins = pio_n * 8 + 2 * sm;
    sm_config_set_out_pins(&c, out_pins, 1);
    sm_config_set_sideset_pins(&c, out_pins + 1);
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
// Clk per PIO bit: 8
// Clk per loop: 128 * 8 = 1024
// Avg clk per channel per loop: 1024 / 16 = 8 * 8 = 64
// Oversampling: 420M / 44100 / 1024 ~= 9.3
// Sample step per loop: 44100 * 1024 / 420M ~= 0.107
// -//- * 65536: 7046 + 1346/3125

uint32_t kSampleStepInt = 7046;
uint32_t kSampleStepRem = 1346;
uint32_t kSampleStepDiv = 3125;

#define MAX_TICK 0xFFFFFF
#define DEBUG_CLK 0
#define CLK_PER_LOOP (1024-0)

uint16_t samples[256 * 16 + 16];

static uint32_t precomputed[129 * 4] = {
    0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
    0x0,        0x100,      0x0,        0x10000000, 0x0,        0x80000,
    0x0,        0x2000800,  0x0,        0x1000,     0x2,        0x20000000,
    0x0,        0x110000,   0x100200,   0x9,        0x1000,     0x0,
    0x82,       0x80000000, 0x20008000, 0x8000,     0x8000,     0x4004000,
    0x4000,     0x88000001, 0x10000400, 0x200010,   0xA00,      0x10002000,
    0x800000,   0x1001000,  0x4000002,  0x2014002,  0x4400,     0x8100,
    0x7202000,  0x80,       0x46041,    0x4000000,  0x50000200, 0x108,
    0x802084,   0x8008000,  0x3002,     0xC0000200, 0x60000060, 0x5048,
    0x4002000,  0x408004,   0x280100,   0x2C000,    0x820020,   0x88080900,
    0x0,        0x10AC0C00, 0x20100082, 0x6001100,  0x408000,   0x180020,
    0x50244401, 0xC084000,  0x41400000, 0x60040,    0x4020008,  0x84229810,
    0x60001010, 0x21001002, 0x41604001, 0x804A0,    0x80840048, 0x2800000,
    0x2008820,  0x25488005, 0x4200800,  0x5080006,  0x4040054,  0x22418082,
    0x90020210, 0x400840,   0x84001808, 0x38680048, 0x2220420C, 0x2020411,
    0x86100100, 0x40022C00, 0x80028BC4, 0x40260041, 0x20000000, 0x40104528,
    0x34100010, 0xAC420042, 0xB0200040, 0x51804400, 0x10004218, 0xC0440000,
    0x3A746018, 0xC20200,   0x1A042804, 0x41880218, 0x20280021, 0xE0261000,
    0x42100010, 0x544004A,  0x4890448,  0xEE202008, 0x204B2C08, 0x21404002,
    0xC014080,  0x88215806, 0x8AA0068,  0x2B0CC008, 0x12401A08, 0x18108020,
    0x90842003, 0x1C463010, 0x43051021, 0x22002122, 0x309920,   0xC340C299,
    0x10028280, 0x90085500, 0x80202800, 0x400503A,  0x1580CC16, 0x30495422,
    0x37450440, 0x80D80008, 0x16040A40, 0x8504D188, 0x200A400,  0xE8D06453,
    0x1804CF1,  0x550440,   0xB02130C0, 0xA4608F00, 0x404062C,  0x136708,
    0x440C3904, 0x40010121, 0x3C03840,  0x41E90F85, 0x1890481,  0x8099950,
    0x45010B9,  0x988625A1, 0x20305220, 0x4E129B8,  0x48035222, 0xC910011B,
    0x26005418, 0x68106052, 0x40911E9,  0x99B06A1,  0x20139C25, 0x48052A00,
    0x98109990, 0x82E51451, 0xC43D020C, 0x8A980648, 0xA8586408, 0x480B8A88,
    0x8264601,  0x2E0268EB, 0x80D80084, 0x2B059A83, 0x2440042F, 0x16678058,
    0x47092089, 0x2BF04A80, 0x80D8A102, 0xE8782C9,  0x554E4080, 0xD189651,
    0xB360913D, 0x118C08A5, 0xAB1C0B20, 0x22700620, 0x48283886, 0xD201B02C,
    0x15006345, 0x43D69E21, 0x322641B,  0x2BDAB342, 0x6C521820, 0xA59350,
    0x8059503B, 0x8922228C, 0x9628406,  0x4721BD3D, 0xCAE06916, 0x2A8A2660,
    0x41095478, 0x838C657,  0x5056E9A1, 0x51A4229E, 0xA8BB0498, 0x92618188,
    0xCB017ECA, 0x2870FF9,  0x2A47A10A, 0x72004850, 0xBE3B0A4A, 0x3B9B2253,
    0xC972010,  0x24AA88E,  0x5818C8C,  0xF57B920C, 0x151B0D84, 0x1C896D23,
    0xA8E1E898, 0xE8D56A54, 0x80AE302,  0xC14F4B89, 0x53CAA8B3, 0x863709C6,
    0x418D5433, 0x134811CB, 0x6873E208, 0xB3A04A60, 0x4DF29278, 0xE4B09974,
    0x9916CD1,  0x6BD0204B, 0xA1F482A6, 0x4997C3B6, 0xA22D4927, 0x359F3D31,
    0x63806CB0, 0xAC29A26A, 0x893A6B84, 0x9AA0BB88, 0x115AC0C6, 0x4F561D77,
    0xC3D4F831, 0xC8F31F71, 0x248A3443, 0xB4069373, 0x51A0A29A, 0xE6DA81A2,
    0x6F0A6AB4, 0xDE5B2536, 0x779DC346, 0x454B02CE, 0x1FA071BD, 0x4E518953,
    0x242B81BD, 0xAEF61E79, 0x8653CE4C, 0xC521B711, 0xF1657E03, 0x24E7F436,
    0x14EAF476, 0x64574423, 0xE6265B43, 0x3D727646, 0xEB091955, 0xB96BB960,
    0x21B697E9, 0xDDD5D262, 0x7DD21636, 0x4A68911F, 0xD97FF83D, 0x31938459,
    0xDE7500AC, 0xE741B715, 0x3AED11F5, 0xF5C3A33E, 0x188512E2, 0x8537E6DF,
    0x5BB52BF7, 0xD9B4A61A, 0x620F8AF0, 0xB5D10FE6, 0xF7FDA6F1, 0xE53E88D0,
    0x1F696E4,  0xB94623ED, 0x9FC42CD6, 0x30F13FBF, 0x6EEB1A6A, 0x305E4F2B,
    0x5F9648FE, 0x37D5F2C1, 0x483ABC2B, 0xFEEBBC6,  0xE77A8737, 0x56C3FEBE,
    0x8C9A8B55, 0xFC929A36, 0xFCD4126F, 0xEAACB5EB, 0x3FBB33F4, 0xE8A18C9B,
    0x3F47B351, 0xA30B60EE, 0x6FF65D9A, 0x7BEC3CEB, 0xAB0C7B35, 0xBAB92CBC,
    0xCEEE672E, 0x6D76A57F, 0xFFEEE66D, 0x5AF5CB70, 0xD950B206, 0xA5BFDFB4,
    0xE57E555B, 0x80EEAFE7, 0xA9B70295, 0x77EFB7F8, 0xA15FDF4D, 0x5F2BA8BE,
    0x79CFFD66, 0xBCF97093, 0x77FFA277, 0xB63CF73A, 0x39DDCF8A, 0xE6F6B30A,
    0xA5AB8C7E, 0x3DBDFEBF, 0xFB7D849F, 0xEB6610F3, 0xCADDDAE6, 0xFB8AFFBD,
    0x262AFFAB, 0x67769C9D, 0x6BDB7FAD, 0x725AF73F, 0x4DEB8DA9, 0xAF90FDBB,
    0x75AE747F, 0x6ABF739F, 0xF0F6F7C4, 0x7EC39DCF, 0x18FBE7DE, 0x7EFA52FF,
    0x2F9AF1EF, 0xDFF587E0, 0x7F7F3FFB, 0xECB6DB09, 0x3B2AAFEF, 0x3F66CB97,
    0xFE7FD7CF, 0xBECFE37E, 0x7B6FFBD1, 0xD8E4612E, 0xB78C7CC7, 0xDBFFFCF6,
    0xBFE5E8F6, 0xFBF660F6, 0xDDBBF371, 0x62BB1FFD, 0xCF747E7D, 0x98EEDFFF,
    0xFF503FE7, 0x56EDF2E7, 0x3EE8FE7F, 0xDCBABFDF, 0x9FF3FDDF, 0x56DF4FFF,
    0xDAB39DAF, 0xBACFBCB1, 0x57FFFEFD, 0xC3EE9FD7, 0xCB3CA7FD, 0xBE7B7C79,
    0x3B577DDE, 0xFFCDFD5A, 0xCFE2FDDF, 0x9FBAFE1F, 0xE596EE87, 0x9E7EDFCE,
    0xF73C7FDF, 0xFFB6FEEF, 0xE2FFF7BD, 0xB7FFDFEC, 0x7746B963, 0xEBBFF7BE,
    0x7769D7EF, 0xFFFDFBA8, 0xBE3D6E2B, 0x7F7FF7FB, 0x5FB67BE6, 0xFDF6D2FD,
    0xDDFFE9FD, 0xFFBBFCE3, 0xDE99DF76, 0xFF7FFFE9, 0x9EF76DDE, 0xEEF6BFBD,
    0xD94FBEDC, 0xFF2FDFBC, 0xFFF7C6FF, 0xFBBEFEFC, 0xCD7FBBEF, 0xFBFFF797,
    0xFFDB7D7F, 0x99DBCABF, 0xBE5BE8F6, 0x5FDF5EFF, 0xFFEFEFFF, 0xFBA7FC7D,
    0xDFFE6FFF, 0xEFFDFBC9, 0xFFEED77F, 0xF69BD977, 0x6F77FFDF, 0xBFDCBFBE,
    0xB3FCDBFB, 0xFDDFF7F6, 0xFFEE77FF, 0xDB7DFEBE, 0x6F71EF1F, 0xFFEFF5FF,
    0x7DFFEDBF, 0xF5F4FFFF, 0x9F67FB7B, 0xDFF5FEDF, 0x7FFFFFD1, 0xFFB77E4D,
    0x6FFEDEBF, 0xEFFAFFFF, 0x7B673FF3, 0xF7FE9EFF, 0xFFEFFDFF, 0xF5FF7FDD,
    0xDFFFE7FE, 0xF6FCFF3F, 0xFBA3FF7F, 0xFB7FFDFD, 0xDF7BBD6F, 0xFFFFD5FF,
    0xFF3EEFBF, 0xF7BFFFF9, 0xBB3FFFBF, 0xFFBEFF7F, 0xFFEAFB7F, 0xE9DBFFFF,
    0x3DF7BFBE, 0xDF7EEFD7, 0xFFF7FEFF, 0xFFFAFFFF, 0xF2EFBF6F, 0xFF1FDFFB,
    0xFFEFFF7F, 0xBDFFFFFF, 0xFDFBBFFF, 0xFFFDF77F, 0x6FF7DFFE, 0x7F7FFFCF,
    0xDEFFFFFF, 0xFFFBFFCF, 0xFCBFFFFB, 0xFFFBB3FE, 0xDFBFFFFF, 0xEEFFF9EF,
    0xFFFF77CF, 0xFFFF5FFF, 0xFFFFF7FF, 0xFFBFDFDB, 0xFF7BFDFB, 0xEFFFDFF7,
    0xEFFFDFA7, 0xBFFFFFFF, 0xFFFBFFF2, 0xFF7FFFFF, 0xFFFFFFFE, 0x3FFFFBFE,
    0xFF7F7FFD, 0xFFFFFFEB, 0xFDFEFFFF, 0xFFDFFF7F, 0xFFFFFFFF, 0xFFDFFBCB,
    0xFFFFDFFF, 0xD7FFFF7B, 0xFDFFFFFF, 0xFEFF7FFF, 0xFFFF7FFF, 0x7FFFFFFF,
    0xF7FFDFFF, 0xFFFFEFF6, 0xFFFFFFF7, 0xFFFEFFFE, 0xFFFFFFFF, 0xEFFDFDFF,
    0xF7FFFFFF, 0xFFFFF7FF, 0x9DFFFFFF, 0xFFFFFFFF, 0xFF7FFFFF, 0xFFFFFDFF,
    0xEFFFFFEF, 0xFFFFFFFF, 0xFFFFFFF7, 0xFFFFFDFF, 0xFFF7FFFF, 0xFFFFFFFF,
    0xFFFFFF7F, 0xBFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFBF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

static uint32_t precomputed_[129 * 4] = {
    0x0,        0x0,        0x0,        0x0,        0x1,        0x0,
    0x0,        0x0,        0x3,        0x0,        0x0,        0x0,
    0x7,        0x0,        0x0,        0x0,        0xF,        0x0,
    0x0,        0x0,        0x1F,       0x0,        0x0,        0x0,
    0x3F,       0x0,        0x0,        0x0,        0x7F,       0x0,
    0x0,        0x0,        0xFF,       0x0,        0x0,        0x0,
    0x1FF,      0x0,        0x0,        0x0,        0x3FF,      0x0,
    0x0,        0x0,        0x7FF,      0x0,        0x0,        0x0,
    0xFFF,      0x0,        0x0,        0x0,        0x1FFF,     0x0,
    0x0,        0x0,        0x3FFF,     0x0,        0x0,        0x0,
    0x7FFF,     0x0,        0x0,        0x0,        0xFFFF,     0x0,
    0x0,        0x0,        0x1FFFF,    0x0,        0x0,        0x0,
    0x3FFFF,    0x0,        0x0,        0x0,        0x7FFFF,    0x0,
    0x0,        0x0,        0xFFFFF,    0x0,        0x0,        0x0,
    0x1FFFFF,   0x0,        0x0,        0x0,        0x3FFFFF,   0x0,
    0x0,        0x0,        0x7FFFFF,   0x0,        0x0,        0x0,
    0xFFFFFF,   0x0,        0x0,        0x0,        0x1FFFFFF,  0x0,
    0x0,        0x0,        0x3FFFFFF,  0x0,        0x0,        0x0,
    0x7FFFFFF,  0x0,        0x0,        0x0,        0xFFFFFFF,  0x0,
    0x0,        0x0,        0x1FFFFFFF, 0x0,        0x0,        0x0,
    0x3FFFFFFF, 0x0,        0x0,        0x0,        0x7FFFFFFF, 0x0,
    0x0,        0x0,        0xFFFFFFFF, 0x0,        0x0,        0x0,
    0xFFFFFFFF, 0x1,        0x0,        0x0,        0xFFFFFFFF, 0x3,
    0x0,        0x0,        0xFFFFFFFF, 0x7,        0x0,        0x0,
    0xFFFFFFFF, 0xF,        0x0,        0x0,        0xFFFFFFFF, 0x1F,
    0x0,        0x0,        0xFFFFFFFF, 0x3F,       0x0,        0x0,
    0xFFFFFFFF, 0x7F,       0x0,        0x0,        0xFFFFFFFF, 0xFF,
    0x0,        0x0,        0xFFFFFFFF, 0x1FF,      0x0,        0x0,
    0xFFFFFFFF, 0x3FF,      0x0,        0x0,        0xFFFFFFFF, 0x7FF,
    0x0,        0x0,        0xFFFFFFFF, 0xFFF,      0x0,        0x0,
    0xFFFFFFFF, 0x1FFF,     0x0,        0x0,        0xFFFFFFFF, 0x3FFF,
    0x0,        0x0,        0xFFFFFFFF, 0x7FFF,     0x0,        0x0,
    0xFFFFFFFF, 0xFFFF,     0x0,        0x0,        0xFFFFFFFF, 0x1FFFF,
    0x0,        0x0,        0xFFFFFFFF, 0x3FFFF,    0x0,        0x0,
    0xFFFFFFFF, 0x7FFFF,    0x0,        0x0,        0xFFFFFFFF, 0xFFFFF,
    0x0,        0x0,        0xFFFFFFFF, 0x1FFFFF,   0x0,        0x0,
    0xFFFFFFFF, 0x3FFFFF,   0x0,        0x0,        0xFFFFFFFF, 0x7FFFFF,
    0x0,        0x0,        0xFFFFFFFF, 0xFFFFFF,   0x0,        0x0,
    0xFFFFFFFF, 0x1FFFFFF,  0x0,        0x0,        0xFFFFFFFF, 0x3FFFFFF,
    0x0,        0x0,        0xFFFFFFFF, 0x7FFFFFF,  0x0,        0x0,
    0xFFFFFFFF, 0xFFFFFFF,  0x0,        0x0,        0xFFFFFFFF, 0x1FFFFFFF,
    0x0,        0x0,        0xFFFFFFFF, 0x3FFFFFFF, 0x0,        0x0,
    0xFFFFFFFF, 0x7FFFFFFF, 0x0,        0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x0,        0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x1,        0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x3,        0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x7,        0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0xF,        0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x1F,       0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x3F,       0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x7F,       0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFF,       0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x1FF,      0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x3FF,      0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x7FF,      0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0xFFF,      0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x1FFF,     0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x3FFF,     0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x7FFF,     0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFF,     0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFF,    0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x3FFFF,    0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFF,    0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFF,    0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x1FFFFF,   0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x3FFFFF,   0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFF,   0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFF,   0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFFFF,  0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x3FFFFFF,  0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x7FFFFFF,  0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFF,  0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFFFFF, 0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0x3FFFFFFF, 0x0,        0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFFFF, 0x0,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x0,        0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x1,        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x3,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7,        0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xF,        0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x1F,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x3F,       0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x7F,       0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x1FF,      0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x3FF,      0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFF,      0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x1FFF,     0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x3FFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFF,     0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFF,     0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x3FFFF,    0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x7FFFF,    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFFF,   0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x3FFFFF,   0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFF,   0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x1FFFFFF,  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x3FFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x7FFFFFF,  0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0xFFFFFFF,  0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x1FFFFFFF,
    0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0x3FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF,
    0xFFFFFFFF, 0x7FFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};

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
    // Wait until voltage is stable.
    for (uint32_t i = 0; i < 5000000; ++i) {
      NOP;
    }
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
    samples[i * 16 + 1] = sinw[i] / 2 + 32768 - (32768 / 2);
    samples[i * 16 + 2] = sinw[i] / 2 + 32768 - (32768 / 2) - 16384;
    samples[i * 16 + 3] = sinw[i] / 2 + 32768 - (32768 / 2) + 16384;
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
    pio0->fdebug = 0xFFFF0000;
    pio1->fdebug = 0xFFFF0000;
  }
}
