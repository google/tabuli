#!/bin/bash
export PICCOLO_ROOT=`pwd`
export PICO_SDK_PATH=`realpath ../pico-sdk`
export PICO_BOARD=pico
cd $PICO_SDK_PATH
git submodule update --init
cd $PICCOLO_ROOT

rm -rf ./build-host
rm -rf ./build-target
mkdir -p ./build-target/deps

python3 make_payload.py

cmake -B build-host ./host -DCMAKE_EXPORT_COMPILE_COMMANDS=1
# --graphviz=./build-target/deps/target.dot
cmake -B build-target ./target -DCMAKE_EXPORT_COMPILE_COMMANDS=1
