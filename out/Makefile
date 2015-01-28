###############################################################################
# Makefile for the project nrf_comm
###############################################################################

## General Flags
PROJECT = nrf_comm
MCU = atmega328p
TARGET = nrf_comm.elf
CESTA=/usr/local/CrossPack-AVR/bin/
CC = $(CESTA)avr-c++

CPP = $(CESTA)avr-c++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -DF_CPU=16000000UL -O1 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=nrf_comm.map


## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Include Directories
#INCLUDES = -I"C:\Users\honza_be\Documents\..\..\..\WinAVR-20100110\avr\include" -I C:\Honza\_nrf24l01\
INCLUDES = 

## Library Directories
#LIBDIRS = -L"C:\WinAVR-20100110\avr\lib" 
LIBDIRS =

## Objects that must be built in order to link
OBJECTS = nrf_comm.o arduino_simple.o Mirf.o spilib.o 

## Objects explicitly added by the user
LINKONLYOBJECTS = 

## Build
all: clean $(TARGET) nrf_comm.hex nrf_comm.eep nrf_comm.lss avr-size
	
touch:	
	touch ../nrf_comm.c
	
## Compile
nrf_comm.o: ../nrf_comm.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

arduino_simple.o: ../arduino_simple.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

spilib.o: ../spilib.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

Mirf.o: ../Mirf.cpp
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

##Link
$(TARGET): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) $(LINKONLYOBJECTS) $(LIBDIRS) $(LIBS) -o $(TARGET)

%.hex: $(TARGET)
	$(CESTA)avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $< $@

%.eep: $(TARGET)
	$(CESTA)avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: $(TARGET)
	$(CESTA)avr-objdump -h -S $< > $@

avr-size:
	$(CESTA)avr-size --mcu=$(MCU) --format=avr $(TARGET)

## Clean target
.PHONY: clean
clean:
	-rm -rf $(OBJECTS) nrf_comm.elf dep/* nrf_comm.hex nrf_comm.eep nrf_comm.lss nrf_comm.map


## Other dependencies
-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

