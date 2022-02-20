# How to build an Install

This document describe how to build firmware and burn on MUXSAN can bridge

## On Linux

### Install AVR compiler and dependencies
    sudo apt-get install gcc-avr binutils-avr avr-libc
### Build

    #> cd <clone repo>/leaf-can-bridge-3-port-CCS/Debug

clean all win pre complied files to avoid issue like
the following one
https://github.com/dalathegreat/Nissan-LEAF-CCS/pull/2#issuecomment-1046327328

    #> make -f Makefile.multiOS clean

build

    #> make -f Makefile.multiOS all

### Burn

    #> avrdude -c avrispmkII -p ATMega32c4 -P usb -e -U flash:w:<your hex file>

NOTE: You should use the correct **-c** option depending on avr flasher

## On OSX
Not tested, please refer to Build on Linux guide

## On Window