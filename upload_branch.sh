#!/bin/bash
cmake --build build-target -j 16 && cp -v ./build-target/branch.uf2 /Volumes/RPI-RP2/
