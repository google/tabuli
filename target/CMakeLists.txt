cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(piccolo-target C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(BUILD_PICCOLO 0)
set(BUILD_FT1248 1)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()
# Initialize the SDK
pico_sdk_init()

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (BUILD_PICCOLO) 
  add_executable(piccolo
    piccolo.c
    payload.s
  )
  pico_set_binary_type(piccolo copy_to_ram)
  pico_define_boot_stage2(piccolo-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(piccolo-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(piccolo piccolo-div4)
  target_link_libraries(piccolo pico_stdlib)
  pico_add_extra_outputs(piccolo)
endif()

if (BUILD_FT1248)
  add_executable(ft1248
    ft1248.c
    sd1306.c
  )

  pico_generate_pio_header(ft1248 ${CMAKE_CURRENT_LIST_DIR}/ft1248.pio)

  pico_set_binary_type(ft1248 copy_to_ram)
  pico_define_boot_stage2(ft1248-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(ft1248-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(ft1248 ft1248-div4)
  target_link_libraries(ft1248
    hardware_i2c
    hardware_pio
    pico_multicore
    pico_stdlib
  )
  pico_add_extra_outputs(ft1248)
endif()
