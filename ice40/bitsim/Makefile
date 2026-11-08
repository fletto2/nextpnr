# make ASC=X.asc PCF=x.pcf [O=build] [ACT=act.txt | PROFILE="EV OUT --clk50 PS [--mem ..]"]
# X.asc from `iceunpack X.bin X.asc`; . /opt/oss-fpga/env.sh first
O ?= build
ASC ?= vera.asc
PCF ?= vera_module.pcf
CFLAGS ?= -O3 -march=native -fno-tree-slp-vectorize -Wall -Wextra
# net numbers are per image, an act.txt from another image is a bad schedule (40% slower here)
ACT ?= $(if $(PROFILE),$(O)/act.txt)
GENFLAGS ?= $(if $(ACT),--act $(ACT))

all: $(O)/replay $(O)/memorder

# the --mem order: which SPRAM cell each RTL bank was placed in, read out of the image
$(O)/memorder: $(O)/bitsim_gen.c memmap.c bitsim.h
	$(CC) -O2 -Wall -Wextra -I. -I$(O) -o $(O)/memmap $(O)/bitsim_gen.c memmap.c
	$(O)/memmap > $@

$(O)/x.net: $(ASC) asc2net.py
	mkdir -p $(O)
	python3 asc2net.py $(ASC) $@ --pcf $(PCF)

$(O)/bitsim_gen.c: $(O)/x.net gen.py runtime.inc $(ACT)
	python3 gen.py $< $@ $(GENFLAGS)

# any trace works as stimulus, placement does not matter to it, so --nocheck
$(O)/act.txt: $(O)/x.net gen.py runtime.inc replay.c
	mkdir -p $(O)/count
	python3 gen.py $< $(O)/count/bitsim_gen.c --part 1 --count
	$(CC) -O2 -march=native -DBS_COUNT -I. -I$(O)/count -o $(O)/count/replay $(O)/count/bitsim_gen.c replay.c
	cd $(O)/count && ./replay $(abspath $(firstword $(PROFILE))) $(abspath $(word 2,$(PROFILE))) $(wordlist 3,99,$(PROFILE)) --nocheck
	cp $(O)/count/act.txt $@

$(O)/replay: $(O)/bitsim_gen.c replay.c bitsim.h
	$(CC) $(CFLAGS) -I. -I$(O) -o $@ $(O)/bitsim_gen.c replay.c

# x16emu with VERA running from this image: $(O)/emu/x16emu
emu: $(O)/bitsim_gen.c
	cmake -S x16emu-bitsim -B $(O)/emu -DCMAKE_BUILD_TYPE=Release -DBITSIM_O=$(abspath $(O))
	cmake --build $(O)/emu -j8

# cell models against yosys' cells_sim.v; needs $(O)/x.net for the image's MAC configurations
test: $(O)/x.net
	python3 tests/mactest.py $(O)/x.net
	python3 tests/ramtest.py

clean:
	rm -rf $(O)
.PHONY: all emu test clean
