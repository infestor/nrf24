###############################################################################
# Makefile for the project nrf_comm
###############################################################################

## General Flags
PROJECT = nrf_comm
MCU = atmega328p
PRIPONA = .elf
TARGET = nrf_comm.elf
TARGET_MASTER = nrf_comm_master.elf

OUTDIR=bin
$(shell mkdir -p $(OUTDIR) >/dev/null)

#CESTA=/usr/local/CrossPack-AVR/bin/
#CESTA="C:/WinAVR-20100110\\bin\\"
CESTA =

CC = $(CESTA)avr-c++
CPP = $(CESTA)avr-c++

## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU)
#COMMON += -D_DEBUG_ 

## Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -g -gdwarf-2 -DF_CPU=16000000UL -O2
CFLAGS += -ffreestanding -fno-tree-scev-cprop -mcall-prologues 
CFLAGS += -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums -fno-jump-tables
CFLAGS += -fdata-sections -ffunction-sections

# handling header file dependency
DEPDIR := $(OUTDIR)/dep
$(shell mkdir -p $(DEPDIR) >/dev/null)
DEPFLAGS = -MMD -MP -MT $@ -MF $(DEPDIR)/$*.Td
POSTCOMPILE = mv -f $(DEPDIR)/$*.Td $(DEPDIR)/$*.d

## Linker flags
LDFLAGS = $(COMMON)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--relax
LDFLAGS_MASTER = -Wl,-u,vfprintf
#LDFLAGS += -Wl,-Map=nrf_comm.map

## Objects explicitly added by the user
LIBS = -lprintf_flt -lm

## Include Directories
#INCLUDES = -I"C:/WinAVR-20100110/avr/include" -I"C:/Honza/_nrf24l01"
INCLUDES = 

## Library Directories
#LIBDIRS = -L"C:/WinAVR-20100110/avr/lib" 
LIBDIRS =

## Objects that must be built in order to link
COMMON_OBJECTS = arduino_simple.o Mirf.o spilib.o
OBJECTS = nrf_comm.o $(COMMON_OBJECTS) onewire.o ds18x20.o
OBJECTS_MASTER = nrf_comm_master.o $(COMMON_OBJECTS)

OBJ_PATH = $(patsubst %,$(OUTDIR)/%,$(OBJECTS))
OBJ_PATH_MASTER = $(patsubst %,$(OUTDIR)/%,$(OBJECTS_MASTER))

## ------------------------------------------------------------------
## Intel Hex file production flags
HEX_FLASH_FLAGS = -R .eeprom -R .fuse -R .lock -R .signature
HEX_EEPROM_FLAGS = -j .eeprom
HEX_EEPROM_FLAGS += --set-section-flags=.eeprom="alloc,load"
HEX_EEPROM_FLAGS += --change-section-lma .eeprom=0 --no-change-warnings

## Assembly specific flags
ASMFLAGS = $(COMMON)
ASMFLAGS += $(CFLAGS)
ASMFLAGS += -x assembler-with-cpp -Wa,-gdwarf2

## ==================================================================
## BUILD
## ==================================================================
all: $(TARGET) $(TARGET_MASTER) nrf_comm.hex nrf_comm_master.hex nrf_comm.lss nrf_comm.size nrf_comm_master.size
	
## Compile
$(OUTDIR)/%.o: %.c $(DEPDIR)/%.d
	$(CC) $(INCLUDES) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<
	$(POSTCOMPILE)

$(OUTDIR)/%.o: %.cpp $(DEPDIR)/%.d
	$(CC) $(INCLUDES) $(CFLAGS) $(DEPFLAGS) -c -o $@ $<
	$(POSTCOMPILE)

##Link
$(TARGET): $(OBJ_PATH)
	$(CC) $(LDFLAGS) $(OBJ_PATH) $(LIBDIRS) -o $(OUTDIR)/$(TARGET)

$(TARGET_MASTER): $(OBJ_PATH_MASTER)
	$(CC) $(LDFLAGS) $(LDFLAGS_MASTER) $(OBJ_PATH_MASTER) $(LIBDIRS) $(LIBS) -o $(OUTDIR)/$(TARGET_MASTER)

%.hex: %.elf
	$(CESTA)avr-objcopy -O ihex $(HEX_FLASH_FLAGS)  $(OUTDIR)/$< $(OUTDIR)/$@

%.eep: %.elf
	$(CESTA)avr-objcopy $(HEX_EEPROM_FLAGS) -O ihex $< $@ || exit 0

%.lss: %.elf
	$(CESTA)avr-objdump -h -S $(OUTDIR)/$< > $(OUTDIR)/$@

%.size: %.elf
	$(CESTA)avr-size --mcu=$(MCU) --format=avr $(OUTDIR)/$<

## Clean target
.PHONY: clean
clean:
	rm -rf $(OUTDIR)/$(TARGET) $(OUTDIR)/$(TARGET_MASTER) $(DEPDIR)/*.d $(OUTDIR)/*.o $(OUTDIR)/*.hex $(OUTDIR)/*.eep $(OUTDIR)/*.lss $(OUTDIR)/*.map

## Other dependencies
$(DEPDIR)/%.d: ;
.PRECIOUS: $(DEPDIR)/%.d

#-include $(shell mkdir dep 2>NUL) $(wildcard dep/*)
SRCS = $(wildcard *.c) $(wildcard *.cpp)
-include $(patsubst %,$(DEPDIR)/%.d,$(basename $(SRCS)))
