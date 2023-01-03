cmake_minimum_required(VERSION 3.12)

project(piccolo-host C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(BUILD_PUSH 1)
set(BUILD_PUSH_ASYNC 1)

find_package(PkgConfig)
pkg_check_modules(LIBUSB libusb-1.0)

set(CMAKE_C_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

if (BUILD_PUSH)
  add_executable(push
    push.cc
  )
  target_include_directories(push PRIVATE
    ${LIBUSB_INCLUDE_DIRS}
  )
  target_link_options(push PUBLIC ${LIBUSB_LDFLAGS})
endif()

if (BUILD_PUSH_ASYNC)
  add_executable(push_async
    push_async.cc
  )
  target_include_directories(push_async PRIVATE
    ${LIBUSB_INCLUDE_DIRS}
  )
  target_link_options(push_async PUBLIC ${LIBUSB_LDFLAGS})
endif()
