#!/bin/bash

set -e

mkdir -p build

pushd build > /dev/null

cmake -DBOARD=nrf52840_pca10056 -GNinja ..

ninja

popd > /dev/null
