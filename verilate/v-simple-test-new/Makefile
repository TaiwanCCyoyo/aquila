# ======================================================================
#  This Makefile builds a test application *.ebf file for Aquila.
#
#  Chun-Jen Tsai, Dec/09/2019
# ======================================================================

CROSS = riscv32-unknown-elf
CCPATH = $(RISCV)/bin

CC = $(CCPATH)/$(CROSS)-gcc
AS = $(CCPATH)/$(CROSS)-as
LD = $(CCPATH)/$(CROSS)-ld
OC = $(CCPATH)/$(CROSS)-objcopy
OD = $(CCPATH)/$(CROSS)-objdump

LIBC = ../../sw/elibc
LIBV = ./v
LIBP = ./p
LIBMACROS = ./macros/scalar

CCFLAGS = -Wall -I$(LIBC) -I$(LIBV) -I$(LIBMACROS) -I$(LIBP) -fomit-frame-pointer -fno-builtin -march=rv32imf -mstrict-align -mabi=ilp32 
ASFLAGS = -march=rv32imf -I$(LIBV) -I$(LIBMACROS) -I$(LIBP)
LDFLAGS = -T$(PROJ).ld -Map=$(PROJ).map # -Ttext=0
LD_SOFT_FP = -L$(RISCV)/lib/gcc/riscv32-unknown-elf/8.2.0 -lgcc

OCFLAGS = -O binary
ODFLAGS = -d

PROJ = rv32ui-v-simple
OBJS = $(LIBC)/stdio.o $(LIBC)/stdlib.o $(LIBC)/heap_4.o  $(LIBC)/time.o $(LIBC)/uart.o rv32ui/simple.o v/vm.o v/entry.o  $(LIBC)/string.o #v/string.o

all:
	make $(PROJ).ebf

clean:
	rm -rf $(LIBC)/*.o *.o *.bin *.objdump *.map *.ebf *.content *.dump

%.o: %.s $(PROJ).ld
	$(AS) $(ASFLAGS) -c $< -o $@

%.o: %.c $(PROJ).ld
	$(CC) $(CCFLAGS) -c $< -o $@

_$(PROJ).o: $(ASMS) $(OBJS)
	#riscv64-unknown-elf-gcc -march=rv32imf -mabi=ilp32 -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -DENTROPY=0xf7930f7 -std=gnu99 -O2 $(OBJS) -I/home/twccyoyo/riscv/aquila_20200410_verilate/aquila/verilate/v-simple-test/v -I/home/twccyoyo/riscv/aquila_20200410_verilate/aquila/verilate/v-simple-test/macros/scalar -T/home/twccyoyo/riscv/aquila_20200410_verilate/aquila/verilate/v-simple-test/v/link.ld /home/twccyoyo/riscv/aquila_20200410_verilate/aquila/verilate/v-simple-test/v/entry.S /home/twccyoyo/riscv/aquila_20200410_verilate/aquila/verilate/v-simple-test/v/*.c rv32ui/simple.S -o $@
	#riscv64-unknown-elf-objdump --disassemble-all --disassemble-zeroes --section=.text --section=.text.startup --section=.text.init --section=.data _$(PROJ).o > $(PROJ).dump
	$(LD) $(LDFLAGS) $(OBJS) $(LD_SOFT_FP) -o $@


$(PROJ).bin: _$(PROJ).o
	$(OC) $(OCFLAGS) $< $@
	cp _$(PROJ).o ver_test
	$(OD) $(ODFLAGS) $< > $(PROJ).objdump
	$(OD) -s $< > $(PROJ).content

$(PROJ).ebf: $(PROJ).bin
	./create_ebf.sh $<
