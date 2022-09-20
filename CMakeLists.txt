cmake_minimum_required(VERSION 3.12)

set(BUILD_PICCOLO 0)
set(BUILD_FT1248 0)
set(BUILD_PUSH 1)

if (NOT BUILD_PUSH)
  # Pull in SDK (must be before project)
  include(pico_sdk_import.cmake)
endif()

project(piccolo C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (BUILD_PUSH)
  find_package(PkgConfig)
  pkg_check_modules(LIBUSB libusb-1.0)
else()
  if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
  endif()
  # Initialize the SDK
  pico_sdk_init()
endif()

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (BUILD_PICCOLO) 
  add_executable(piccolo
    src/piccolo.c
    src/payload.s
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
    src/ft1248.c
    src/sd1306.c
  )
  pico_set_binary_type(ft1248 copy_to_ram)
  pico_define_boot_stage2(ft1248-div4 ${PICO_DEFAULT_BOOT_STAGE2_FILE})
  target_compile_definitions(ft1248-div4 PRIVATE PICO_FLASH_SPI_CLKDIV=4)
  pico_set_boot_stage2(ft1248 ft1248-div4)
  target_link_libraries(ft1248
    hardware_i2c
    pico_multicore
    pico_stdlib
  )
  pico_add_extra_outputs(ft1248)
endif()

message("${LIBUSB_INCLUDE_DIRS} ${LIBUSB_LDFLAGS}")

if (BUILD_PUSH)
  add_executable(push
    src/push.cc
  )
  target_include_directories(push PRIVATE
    ${LIBUSB_INCLUDE_DIRS}
  )
  target_link_options(push PUBLIC ${LIBUSB_LDFLAGS})
endif()