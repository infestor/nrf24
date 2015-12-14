#!/bin/bash

avrdude -patmega328p -carduino -P\\dev\tty.wchusbserialfa130 -b57600 -D -Uflash:w:nrf_comm.hex:i