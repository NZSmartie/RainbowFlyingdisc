# Bluetooth: Peripheral

## Overview

Application demonstrating the BLE Peripheral role. It has several well-known and
vendor-specific GATT services that it exposes.


## Requirements

* BlueZ running on the host, or
* A board with BLE support

## Building and Running

```bash
export ZEPHYR_TOOLCHAIN_VARIANT=gnuarmemb
export GNUARMEMB_TOOLCHAIN_PATH=~/gcc-arm-none-eabi-7-2018-q2-update/ # path to downloaded toolchain

source ./setup-env.sh # Setups up python virtualenv, and zephyr

./build.sh # invokes cmake nad builds the application

```

## Flashing Example

An example of using OpenOCD with Adafruit's FT232H breakout board to flash the nRF52

> *Note*: `D0` is SWDCLK, `D1` is SWDIO, and `D2` **must** be connected to `D1` with a 470 Ohm (or equivalent) resistor.

```bash
openocd \
    -f interface/ftdi/ft232h-module-swd.cfg `# Use FT232h as our hardware interface` \
    -f target/nrf52.cfg `# the target chip` \
    -c "init" `# Tell OpenOCD we're done with setup` \
    -c "reset halt" `# reset the chip and halt the CPU` \
    -c "flash write_image erase build/zephyr/zephyr.hex" `# Flash our firmware image` \
    -c "reset run" `# reset and run` \
    -c "exit" `# reset and run`
```
