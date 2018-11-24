#!/bin/bash

set -e

mkdir -p build

pushd build > /dev/null

cmake -GNinja ..

ninja

popd > /dev/null
