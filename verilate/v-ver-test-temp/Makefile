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

CCFLAGS = -Wall -I$(LIBC) -fomit-frame-pointer -fno-builtin -march=rv32im -mstrict-align -mabi=ilp32
ASFLAGS = -march=rv32im
LDFLAGS_V = -T$(PROJ)-v.ld -Map=$(PROJ).map # -Ttext=0
LDFLAGS_P = -T$(PROJ)-p.ld -Map=$(PROJ).map
LD_SOFT_FP = -L$(RISCV)/lib/gcc/riscv32-unknown-elf/8.2.0 -lgcc

OCFLAGS = -O binary
ODFLAGS = -d

PROJ = test
OBJS = vm.o handle_trap.o test.o $(LIBC)/stdio.o $(LIBC)/stdlib.o $(LIBC)/heap_4.o $(LIBC)/string.o $(LIBC)/time.o $(LIBC)/uart.o
BOOT_V = boot-v.o
BOOT_P = boot-p.o

all:
	make $(PROJ)-p.ebf
	make $(PROJ)-v.ebf 

clean: 
	rm -rf $(LIBC)/*.o *.o *.bin *.objdump *.map *.ebf *.content $(PROJ)-v $(PROJ)-p

%.o: %.s 
	$(AS) $(ASFLAGS) -c $< -o $@

%.o: %.c
	$(CC) $(CCFLAGS) -c $< -o $@

%-p.o: %.c
	$(CC) $(CCFLAGS) -c $< -o $@

%-v.o: %.c
	$(CC) $(CCFLAGS) -c $< -o $@ -D VIRTUAL_MEMORY_ENABLE

$(PROJ)-v: $(ASMS) $(OBJS) $(BOOT_V)
	$(LD) $(LDFLAGS_V) $(OBJS) $(BOOT_V) $(LD_SOFT_FP) -o $@
	
$(PROJ)-p: $(ASMS) $(OBJS) $(BOOT_P)
	$(LD) $(LDFLAGS_P) $(OBJS) $(BOOT_P) $(LD_SOFT_FP) -o $@

%.bin: %
	$(OC) $(OCFLAGS) $< $@
	$(OD) $(ODFLAGS) $< > $<.objdump
	$(OD) -s $< > $<.content

%.ebf: %.bin
	./create_ebf.sh $<
