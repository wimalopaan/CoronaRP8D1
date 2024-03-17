# sudo avrdude -p attiny85 -C+/home/lmeier/Projekte/wmucpp/avrdude.conf -P usb -c avrisp2 -U lfuse:w:0x62:m -U hfuse:w:0xd7:m -U efuse:w:0xff:m -U flash:w:test00.elf

MCU = atmega328pb
F_OSC = 8000000

MCUFLAGS = -mmcu=$(MCU)
CXXFLAGS += $(MCUFLAGS)
CFLAGS += $(MCUFLAGS) -DF_CPU=$(F_OSC) -Os

CC = /usr/bin/avr-gcc
CXX = /usr/bin/avr-g++
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size
NM = avr-nm
AVRDUDE = avrdude

all: main.elf main.hex

main.elf: main.c

%: %.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS)  $(shell pwd)/$< --output $@ $(LDFLAGS) $(LOADLIBES)

%.elf: %.cc
	$(CXX) $(CPPFLAGS) $(CXXFLAGS)  $(shell pwd)/$< --output $@ $(LDFLAGS) $(LOADLIBES)

%.elf: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) $(shell pwd)/$< --output $@ $(LDFLAGS) $(LOADLIBES)

%.hex: %.elf
	$(OBJCOPY) -O ihex -R .eeprom $(shell pwd)/$< $@
