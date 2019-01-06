cmake_minimum_required(VERSION 3.8.2)

# Tell Zephyr where to find the `boards` directory
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

# Configure zephyr to use our custom board by default
set(BOARD nrf52_flyingdisc)

# Add our own Kconfig
set(KCONFIG_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/Kconfig)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(peripheral)

target_sources(app PRIVATE
  src/main.c
  src/gatt.c
)

# zephyr_library_include_directories($ENV{ZEPHYR_BASE}/samples/bluetooth)
