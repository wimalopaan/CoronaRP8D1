# erase eeprom
# sudo avrdude -p atmega328pb -P usb -c avrisp2  -U flash:w:main.elf -U lfuse:w:0xc6:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m
# preserve eeprom
# sudo avrdude -p atmega328pb -P usb -c avrisp2  -U flash:w:main.elf -U lfuse:w:0xc6:m -U hfuse:w:0xd1:m -U efuse:w:0xff:m
# sudo avrdude -p atmega328pb -P usb -c avrisp2  -U flash:w:main.elf

MCU = atmega328pb
F_OSC = 8000000

MCUFLAGS = -mmcu=$(MCU)
CXXFLAGS += $(MCUFLAGS)
CFLAGS += $(MCUFLAGS) -DF_CPU=$(F_OSC) -O3

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

clean:
	$(RM) *.elf
	$(RM) *.hex

