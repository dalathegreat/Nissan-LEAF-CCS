# How to build an Install

This document describe how to build firmware and burn on MUXSAN can bridge

## On Linux

### Build

#> cd <clone repo>/leaf-can-bridge-3-port-CCS/Debug
#> make -f Makefile.multiOS all

### Burn

#> avrdude -c avrispmkII -p ATMega32c4 -P usb -e -U flash:w:<your hex file>

NOTE: You should use the correct -c option depending on avr flasher

## On OSX
Not tested, please refer to Build on Linux guide

## On Window