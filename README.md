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
