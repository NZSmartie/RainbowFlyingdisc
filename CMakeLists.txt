cmake_minimum_required(VERSION 3.8.2)

# Tell Zephyr where to find the `boards` directory
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

# Configure zephyr to use our custom board by default
set(BOARD nrf52_flyingdisc)

include($ENV{ZEPHYR_BASE}/cmake/app/boilerplate.cmake NO_POLICY_SCOPE)
project(peripheral)

target_sources(app PRIVATE
  src/main.c
  src/mpu6050.c
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/hrs.c
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/bas.c
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/cts.c
  $ENV{ZEPHYR_BASE}/samples/bluetooth/gatt/dis.c
)

zephyr_library_include_directories($ENV{ZEPHYR_BASE}/samples/bluetooth)
