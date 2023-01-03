#!/bin/bash
#cmake --build build-host -j 16 && ./build-host/push
#cmake --build build-host -j 16 && ./build-host/push_async
cmake --build build-host -j 16 && ./build-host/push_async ../cclvi/snd.mux
