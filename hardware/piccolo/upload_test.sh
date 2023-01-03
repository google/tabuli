#!/bin/bash
cmake --build build-target -j 16 && cp -v ./build-target/test.uf2 /Volumes/RPI-RP2/
