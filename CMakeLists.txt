cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(piccolo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_executable(piccolo
  src/piccolo.c
  src/payload.s
)

# pico_set_binary_type(piccolo copy_to_ram)

pico_define_boot_stage2(piccolo-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
target_compile_definitions(piccolo-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
pico_set_boot_stage2(piccolo piccolo-div4)

# pull in common dependencies
target_link_libraries(piccolo pico_stdlib)

# create map/bin/hex file etc.
pico_add_extra_outputs(piccolo)

add_executable(piccolo_wireless
  src/piccolo_wireless.c
)

target_compile_definitions(piccolo_wireless PRIVATE
  WIFI_SSID=$ENV{WIFI_SSID}
  WIFI_PASSWORD=$ENV{WIFI_PASSWORD}
)

pico_enable_stdio_usb(piccolo_wireless 1)
pico_enable_stdio_uart(piccolo_wireless 0)
  
target_include_directories(piccolo_wireless PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/src
)

target_link_libraries(piccolo_wireless
  pico_cyw43_arch_lwip_poll
  pico_stdlib
)
pico_add_extra_outputs(piccolo_wireless)
