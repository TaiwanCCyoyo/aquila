# ======================================================================
#  This Makefile builds the boot code ROM memory file for Xilinx Vivado
#  EDA tools.
#
#  Chun-Jen Tsai, Nov/04/2019
# ======================================================================
#

BUILD_DIR = ./build

FREERTOS_DIR = ../FreeRTOS
FREERTOS_SOURCE_DIR = $(FREERTOS_DIR)/Source

FREERTOS_SRC = \
	$(FREERTOS_SOURCE_DIR)/list.c \
	$(FREERTOS_SOURCE_DIR)/queue.c \
	$(FREERTOS_SOURCE_DIR)/tasks.c \
	$(FREERTOS_SOURCE_DIR)/portable/MemMang/heap_4.c \
	$(FREERTOS_SOURCE_DIR)/timers.c \
#	$(FREERTOS_SOURCE_DIR)/event_groups.c \
	$(FREERTOS_SOURCE_DIR)/croutine.c \
	$(FREERTOS_SOURCE_DIR)/stream_buffer.c \
	

FREERTOS_INC = $(FREERTOS_SOURCE_DIR)/include

FREERTOS_INCLUDES := \
	-I $(FREERTOS_INC)

INTERRUPT_HANDLER = main

FREERTOS_BUILD_DIR = $(BUILD_DIR)/FreeRTOS
FREERTOS_OBJS = $(patsubst %.c,$(FREERTOS_BUILD_DIR)/%.o,$(notdir $(FREERTOS_SRC)))
VPATH += \
	$(FREERTOS_SOURCE_DIR) \
	$(FREERTOS_SOURCE_DIR)/portable/MemMang

# Platform spcific settings for this application _____________________________________________

ARCH = RISC-V
ARCH_PORTABLE_INC = $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)/
ARCH_PORTABLE_SRC = $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)/port.c
ARCH_PORTABLE_ASM = $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)/portASM.S

PORT_OBJS := $(patsubst %.c,$(FREERTOS_BUILD_DIR)/%.o,$(notdir $(ARCH_PORTABLE_SRC)))
PORT_OBJS += $(patsubst %.S,$(FREERTOS_BUILD_DIR)/%.o,$(notdir $(ARCH_PORTABLE_ASM)))
FREERTOS_OBJS += $(PORT_OBJS) 

VPATH += $(FREERTOS_SOURCE_DIR)/portable/GCC/$(ARCH)

CFLAGS = -DportasmHANDLE_INTERRUPT=$(INTERRUPT_HANDLER)

# Set up application source, include, and object files for compilation: ______________________
APP_SRC_DIR = .
APP_SRC = \
	$(APP_SRC_DIR)/main.c \
	$(APP_SRC_DIR)/boot.c \
	$(LIBC)/stdio.c \
	$(LIBC)/stdlib.c \
	$(LIBC)/string.c \
	$(LIBC)/uart.c

APP_INCLUDES = \
	-I ./ \
	-I $(FREERTOS_INC) \
	-I $(ARCH_PORTABLE_INC) \
	-I $(FREERTOS_DIR)/Demo/Common/include \
	-I $(LIBC)

APP_BUILD_DIR = $(BUILD_DIR)/app
APP_OBJS := $(patsubst %.c,$(APP_BUILD_DIR)/%.o,$(notdir $(APP_SRC)))

VPATH += $(APP_SRC_DIR) $(LIBC)

DEMO_COMMON_SRC =  \
#	$(FREERTOS_DIR)/Demo/Common/Minimal/EventGroupsDemo.c \
	$(FREERTOS_DIR)/Demo/Common/Minimal/TimerDemo.c \
	$(FREERTOS_DIR)/Demo/Common/Minimal/TaskNotify.c \
	$(FREERTOS_DIR)/Demo/Common/Minimal/GenQTest.c \
	$(FREERTOS_DIR)/Demo/Common/Minimal/blocktim.c \
	$(FREERTOS_DIR)/Demo/Common/Minimal/dynamic.c \
	$(FREERTOS_DIR)/Demo/Common/Minimal/recmutex.c

DEMO_COMMON_OBJS := $(patsubst %.c,$(APP_BUILD_DIR)/%.o,$(notdir $(DEMO_COMMON_SRC)))
#VPATH += $(FREERTOS_DIR)/Demo/Common/Minimal

# Need to tell FreeRTOS where to find the FreeRTOSConfig.h __________________________________
FREERTOS_INCLUDES += \
	-I ./ \
	-I $(ARCH_PORTABLE_INC) \
	-I $(FREERTOS_DIR)/Demo/Common/include \
	-I $(FREERTOS_SOURCE_DIR)/portable/GCC/RISC-V/chip_specific_extensions/RV32I_CLINT_no_extensions

# List of object files to compile for the system:
OUT_OBJS = \
	$(APP_OBJS) \
	$(DEMO_COMMON_OBJS) \
	$(FREERTOS_OBJS)

BUILD_DIRECTORIES = \
	$(APP_BUILD_DIR) \
	$(FREERTOS_BUILD_DIR)

# RISC-V compile tool
CROSS = riscv32-unknown-elf
CCPATH = $(RISCV)/bin

GCC = $(CCPATH)/$(CROSS)-gcc
AS = $(CCPATH)/$(CROSS)-as
LD = $(CCPATH)/$(CROSS)-ld
OBJCOPY = $(CCPATH)/$(CROSS)-objcopy
OBJDUMP = $(CCPATH)/$(CROSS)-objdump

LIBC = ../elibc

CFLAGS += -Wall -O0 -fomit-frame-pointer -march=rv32im -mstrict-align -fno-builtin -mabi=ilp32
ASMFLAGS = -O0 -march=rv32im -DportasmHANDLE_INTERRUPT=main
LDFLAGS = -O0 -Wl,-Map,"$(BUILD_DIR)/$(PROJ).map" -T$(PROJ).ld -nostartfiles -static  # -Ttext=0
#LD_SOFT_FP = -L/opt/riscv/lib/gcc/riscv32-unknown-elf/8.2.0 -lgcc

PROJ = FreeRTOS_demo
OUT_ELF = $(BUILD_DIR)/$(PROJ).elf

.PHONY: debug clean app_compile debug-app frtos_compile out_elf
all: directories $(OUT_OBJS) $(OUT_ELF)
directories: $(BUILD_DIRECTORIES)
app_compile: directories $(APP_OBJS)
frtos_compile: directories $(FREERTOS_OBJS)
out_elf: directories $(OUT_ELF)

# Compile Object Files ____________________________________________________________________
$(APP_BUILD_DIR)/%.o : %.c
	@echo "[APP Objects] : $@ ______________________________________________________________"
	@echo "Building: $<"
	$(GCC) $(CFLAGS) $(APP_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

$(APP_BUILD_DIR)/%.o : %.S
	@echo "[APP Objects] : $@ ______________________________________________________________"
	@echo "Building: $<"
	$(GCC) $(ASMFLAGS) $(APP_INCLUDES) $(FREERTOS_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

$(FREERTOS_BUILD_DIR)/%.o : %.c
	@echo "[FreeRTOS Objects for .c] : $@ __________________________________________________________"
	@echo "Building: $<"
	$(GCC) $(CFLAGS) $(FREERTOS_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

$(FREERTOS_BUILD_DIR)/%.o : %.S
	@echo "[FreeRTOS Objects for .S] : $@ __________________________________________________________"
	@echo "Building: $<"
	$(GCC) $(ASMFLAGS) $(FREERTOS_INCLUDES) -o $@ -c $<
	@echo "Finished Building: $<"

# Generate ELF ___________________________________________________________________________
$(OUT_ELF): $(OUT_OBJS)
	@echo '___________________________________________________________________________________'
	@echo 'Building target: $@'
	@echo '_______________________________'
	$(GCC) $(LDFLAGS) -o $@ $(OUT_OBJS) $(LD_SOFT_FP)
	$(OBJDUMP) -d -M no-aliases,numeric $@ > ./build/$(PROJ).objdump
	$(OBJCOPY) -O binary ./build/$(PROJ).elf $(PROJ).bin
	./create_ebf.sh $(PROJ).bin
	@echo 'Finished building target: $@'
	@echo ' '

$(PROJ).ebf: $(PROJ).bin
	./create_ebf.sh $<

$(BUILD_DIRECTORIES):
	mkdir -p $@

clean:
	rm -rf *.s *.o *.bin *.objdump *.map *.ebf *.content
	rm -rv $(BUILD_DIR)