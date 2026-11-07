# make ASC=X.asc PCF=x.pcf [ACT=act.txt] [O=build]
# X.asc from `iceunpack X.bin X.asc`; . /opt/oss-fpga/env.sh first
O ?= build
ASC ?= vera.asc
PCF ?= vera_module.pcf
CFLAGS ?= -O3 -march=native -Wall -Wextra
GENFLAGS ?= $(if $(ACT),--act $(ACT))

all: $(O)/replay

$(O)/x.net: $(ASC) asc2net.py
	mkdir -p $(O)
	python3 asc2net.py $(ASC) $@ --pcf $(PCF)

$(O)/bitsim_gen.c: $(O)/x.net gen.py runtime.inc
	python3 gen.py $< $@ $(GENFLAGS)

$(O)/replay: $(O)/bitsim_gen.c replay.c bitsim.h
	$(CC) $(CFLAGS) -I. -I$(O) -o $@ $(O)/bitsim_gen.c replay.c

# x16emu with VERA running from this image: $(O)/emu/x16emu
emu: $(O)/bitsim_gen.c
	cmake -S x16emu-bitsim -B $(O)/emu -DCMAKE_BUILD_TYPE=Release -DBITSIM_O=$(abspath $(O))
	cmake --build $(O)/emu -j8

clean:
	rm -rf $(O)
.PHONY: all emu clean
