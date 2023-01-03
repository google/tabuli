cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(piccolo-target C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(USE_DISPLAY 1)

# 0: RAW
# 1: SD + DITHER
# 2: SD + PWM
set(PICCOLO_PLAY_MODE 1)
add_compile_definitions(branch PRIVATE PICCOLO_PLAY_MODE=${PICCOLO_PLAY_MODE})

set(BUILD_STEM 1)
set(BUILD_BRANCH 1)
set(BUILD_TEST 1)

if (USE_DISPLAY)
  set(DISPLAY_IMPLEMENTATION sd1306.c)
else()
  set(DISPLAY_IMPLEMENTATION void.c)
endif()

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
  message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
# Initialize the SDK
pico_sdk_init()

set(CMAKE_C_FLAGS_RELEASE "-O3 -fno-data-sections")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fno-data-sections")

# This cuts binary size, but disables alarms.
#if (NOT USE_DISPLAY)
#  target_compile_definitions(hardware_timer INTERFACE PICO_TIME_DEFAULT_ALARM_POOL_DISABLED=1)
#endif()

if (BUILD_BRANCH)
  if (PICCOLO_PLAY_MODE EQUAL 0)
    set (PLAY_SRC play_raw.c)
  elseif (PICCOLO_PLAY_MODE EQUAL 1)
    set (PLAY_SRC
      play_sd_dither.c
      sd_patterns.c
    )
  elseif (PICCOLO_PLAY_MODE EQUAL 2)
    set (PLAY_SRC play_sd_pwm.c)
  else()
    message(FATAL_ERROR "Unknown mode")
  endif()

  add_executable(branch
    branch.c
    ${PLAY_SRC}
  )

  pico_generate_pio_header(branch ${CMAKE_CURRENT_LIST_DIR}/sound_raw.pio)
  pico_generate_pio_header(branch ${CMAKE_CURRENT_LIST_DIR}/sound_sd_dither.pio)
  pico_generate_pio_header(branch ${CMAKE_CURRENT_LIST_DIR}/sound_sd_pwm.pio)

  pico_set_binary_type(branch copy_to_ram)
  pico_define_boot_stage2(branch-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(branch-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(branch branch-div4)
  target_link_libraries(branch
    hardware_pio
    hardware_spi
    pico_multicore
    pico_stdlib
  )
  pico_add_extra_outputs(branch)
endif()

if (BUILD_STEM)
  add_executable(stem
    stem.c
    ${DISPLAY_IMPLEMENTATION}
  )

  pico_set_float_implementation(stem none)
  pico_set_double_implementation(stem none)
  pico_set_printf_implementation(stem none)
  # pico_set_bit_ops_implementation(stem compiler)
  pico_set_divider_implementation(stem compiler)
  # pico_set_int64_ops_implementation(stem compiler)

  #pico_enable_stdio_semihosting(stem 0)
  pico_enable_stdio_usb(stem 1)
  pico_enable_stdio_uart(stem 0)

  pico_generate_pio_header(stem ${CMAKE_CURRENT_LIST_DIR}/ft1248.pio)
  pico_generate_pio_header(stem ${CMAKE_CURRENT_LIST_DIR}/pspi.pio)

  pico_set_binary_type(stem copy_to_ram)
  pico_define_boot_stage2(stem-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(stem-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(stem stem-div4)
  target_link_libraries(stem
    hardware_pio
    pico_multicore
    pico_stdlib
  )
  if (USE_DISPLAY)
    target_link_libraries(stem hardware_i2c)
  endif()
  pico_add_extra_outputs(stem)
endif()

if (BUILD_TEST)
  add_executable(test
    sd1306.c
    test.c
  )

  target_compile_definitions(test PRIVATE SD1306_I2C=i2c1)
  target_compile_definitions(test PRIVATE SD1306_BASE_PIN=26)

  pico_set_binary_type(test copy_to_ram)
  pico_define_boot_stage2(test-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(test-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(test test-div4)
  target_link_libraries(test
    hardware_i2c
    hardware_spi
    pico_multicore
    pico_stdlib
  )
  pico_add_extra_outputs(test)
endif()
