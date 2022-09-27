#!/bin/bash
cmake --build build-host -j 16 && ./build-host/push
