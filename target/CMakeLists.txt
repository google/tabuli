cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(piccolo-target C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(BUILD_PICCOLO 1)
set(BUILD_FT1248 1)
set(USE_DISPLAY 1)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
# Initialize the SDK
pico_sdk_init()

set(CMAKE_C_FLAGS_RELEASE "-O3 -g3 -fno-data-sections")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g3 -fno-data-sections")

if (NOT USE_DISPLAY)
  target_compile_definitions(hardware_timer INTERFACE PICO_TIME_DEFAULT_ALARM_POOL_DISABLED=1)
endif()

if (BUILD_PICCOLO) 
  add_executable(piccolo
    piccolo.c
  )

  pico_generate_pio_header(piccolo ${CMAKE_CURRENT_LIST_DIR}/sound.pio)

  pico_set_binary_type(piccolo copy_to_ram)
  pico_define_boot_stage2(piccolo-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(piccolo-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(piccolo piccolo-div4)
  target_link_libraries(piccolo
    hardware_pio
    pico_stdlib
  )
  pico_add_extra_outputs(piccolo)
endif()

if (USE_DISPLAY)
set(DISPLAY_IMPLEMENTATION sd1306.c)
else()
set(DISPLAY_IMPLEMENTATION void.c)
endif()

if (BUILD_FT1248)
  add_executable(ft1248
    ft1248.c
    ${DISPLAY_IMPLEMENTATION}
  )

  pico_set_float_implementation(ft1248 none)
  pico_set_double_implementation(ft1248 none)
  pico_set_printf_implementation(ft1248 none)
  # pico_set_bit_ops_implementation(ft1248 compiler)
  pico_set_divider_implementation(ft1248 compiler)
  # pico_set_int64_ops_implementation(ft1248 compiler)

  pico_enable_stdio_semihosting(ft1248 0)
  pico_enable_stdio_uart(ft1248 0)
  pico_enable_stdio_usb(ft1248 0)

  pico_generate_pio_header(ft1248 ${CMAKE_CURRENT_LIST_DIR}/ft1248.pio)

  pico_set_binary_type(ft1248 copy_to_ram)
  pico_define_boot_stage2(ft1248-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(ft1248-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(ft1248 ft1248-div4)
  target_link_libraries(ft1248
    hardware_pio
    pico_multicore
    pico_stdlib
  )
  if (USE_DISPLAY)
    target_link_libraries(ft1248 hardware_i2c)
  endif()
  pico_add_extra_outputs(ft1248)
endif()
