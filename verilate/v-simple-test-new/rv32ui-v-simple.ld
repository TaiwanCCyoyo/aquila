OUTPUT_ARCH( "riscv" )
ENTRY(_start)

__heap_start = 0x80400000;
__heap_size  = 0x00100000;
__stack_top  = 0x80A00000;
__stack_size = 0x00100000;
__stack_start = __stack_top - __stack_size;

MEMORY
{
    code_ram   (rx!rw) : ORIGIN = 0x80000000, LENGTH = 0x00100000
    data_ram   (rw!x)  : ORIGIN = 0x80020000, LENGTH = 0x00100000
    heap_ram   (rw!x)  : ORIGIN = __heap_start, LENGTH = __heap_size
    stack_ram  (rw!x)  : ORIGIN = __stack_start, LENGTH = __stack_size
}

SECTIONS
{
  . = 0x80000000;
  .text.init : { *(.text.init) }
  . = ALIGN(0x1000);
  .tohost : { *(.tohost) }
  . = ALIGN(0x1000);
  .text : { *(.text) }
  . = ALIGN(0x1000);
  .data : { *(.data) }
  .bss : { *(.bss) }
  _end = .;
}

