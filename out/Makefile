###############################################################################
# Makefile for the project nrf_comm
###############################################################################

## General Flags
PROJECT = nrf_comm
MCU = atmega328p
PRIPONA = .elf
TARGET = nrf_comm.elf
TARGET_MASTER = nrf_comm_master.elf
CESTA=/usr/local/CrossPack-AVR/bin/
#CESTA="C:/WinAVR-20100110\\bin\\"
CESTA =
CC = $(CESTA)avr-g++
#CC = $(CESTA)avr-c++

CPP = $(CESTA)avr-c++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -DF_CPU=16000000UL -O1 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d -fdata-sections -ffunction-sections

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON) -dead_strip -Wl,--gc-sections 
LDFLAGS +=  -Wl,-Map=nrf_comm.map
LDFLAGS_MASTER = $(COMMON)
LDFLAGS_MASTER +=  -Wl,-Map=nrf_comm_master.map

## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature

HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Include Directories
#INCLUDES = -I"C:/WinAVR-20100110/avr/include" -I"C:/Honza/_nrf24l01"
INCLUDES = 

## Library Directories
#LIBDIRS = -L"C:/WinAVR-20100110/avr/lib" 
LIBDIRS =

## Objects that must be built in order to link
OBJECTS = nrf_comm.o arduino_simple.o Mirf.o spilib.o onewire.o ds18x20.o
OBJECTS_MASTER = nrf_comm_master.o arduino_simple.o Mirf.o spilib.o onewire.o ds18x20.o


## Objects explicitly added by the user
LIBS = 

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

onewire.o: ../onewire.c
	$(CC) $(INCLUDES) $(CFLAGS) -c  $<

ds18x20.o: ../ds18x20.c
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

