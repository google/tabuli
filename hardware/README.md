### Overview

This folder contains projects closely related to hardware. E.g. software that
runs on SBC / MCU, as well as software used to communicate with such
units; PCB designs; design-docs; hardware simulation modules.

### Projects

 * piccolo - cast 256 audio streams over USB to `RPi-Pico` MCU; this unit in
   turn streams data to 16 other `RPi-Pico`s, each of those (optionally)
   performs $\Sigma-\Delta$ transform and pushes 16 digital outputs for
   D-class-like amplifiers

