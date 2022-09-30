#!/bin/bash
#cmake --build build-host -j 16 && ./build-host/push
cmake --build build-host -j 16 && ./build-host/push_async
