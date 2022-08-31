#!/bin/bash
export PICCOLO_ROOT=`pwd`
export PICO_SDK_PATH=`realpath ../pico-sdk`
export PICO_BOARD=pico_w
cd $PICO_SDK_PATH
git submodule update --init
cd $PICCOLO_ROOT
rm -rf ./build

python3 make_payload.py

cmake -B build . -DCMAKE_EXPORT_COMPILE_COMMANDS=1
