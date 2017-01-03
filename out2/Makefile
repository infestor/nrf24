###############################################################################
# Makefile for the project nrf_comm
###############################################################################

## General Flags
PROJECT = nrf_comm
MCU = atmega328p
TARGET = nrf_comm_master.elf
CESTA=/usr/local/CrossPack-AVR/bin/
#CESTA = 

#CC = $(CESTA)avr-g++
CC = $(CESTA)avr-g++
CPP = $(CESTA)avr-g++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -DF_CPU=16000000UL -O1 -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-jump-tables
CFLAGS += -fdata-sections -ffunction-sections
#CFLAGS += -MD -MP -MT $(*F).o -MF dep/$(@F).d 
#CFLAGS += -D_DEBUG_ 

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## Linker flags
LDFLAGS = $(COMMON) -dead_strip -Wl,--gc-sections 
LDFLAGS +=  -Wl,-u,vfprintf

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
#LIBDIRS = -L"/usr/local/CrossPack-AVR/avr/lib"
LIBDIRS =

## Objects that must be built in order to link
OBJECTS = nrf_comm_master.o Mirf.o spilib.o arduino_simple.o

## Objects explicitly added by the user
LIBS = -lprintf_flt -lm

## Build
all: clean $(TARGET) nrf_comm_master.hex nrf_comm_master.eep nrf_comm_master.lss avr-size
	
touch:	
	touch ../nrf_comm_master.c
	
## Compile
nrf_comm_master.o: ../nrf_comm_master.c
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
	-rm -rf $(OBJECTS) nrf_comm_master.elf dep/* nrf_comm_master.hex nrf_comm_master.eep nrf_comm_master.lss nrf_comm_master.map


## Other dependencies
#-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)

