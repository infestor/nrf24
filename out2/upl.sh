#!/bin/bash

avrdude -patmega328p -carduino -P/dev/tty.wchusbserialfa130 -b57600 -D -Uflash:w:../bin/nrf_comm_master.hex:i
avrdude -patmega328p -carduino -P/dev/tty.wchusbserialfd120 -b57600 -D -Uflash:w:../bin/nrf_comm_master.hex:i
avrdude -patmega328p -carduino -P/dev/ttyUSB0 -b57600 -D -Uflash:w:../bin/nrf_comm_master.hex:i

