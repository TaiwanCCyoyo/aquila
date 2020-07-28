#include "handle_trap.h"
#include "vm.h"
#define VIRTUAL_MEMORY_ENABLE 0

	.file	"boot.c"
	.option nopic
	.text
	.align	2
	.globl	boot
	.type	boot, @function
boot:
	j handle_reset
	
handle_reset:
	li x1, 0
	li x2, 0
	li x3, 0
	li x4, 0
	li x5, 0
	li x6, 0
	li x7, 0
	li x8, 0
	li x9, 0
	li x10, 0
	li x11, 0
	li x12, 0
	li x13, 0
	li x14, 0
	li x15, 0
	li x16, 0
	li x17, 0
	li x18, 0
	li x19, 0
	li x20, 0
	li x21, 0
	li x22, 0
	li x23, 0
	li x24, 0
	li x25, 0
	li x26, 0
	li x27, 0
	li x28, 0
	li x29, 0
	li x30, 0
	li x31, 0
	lui sp, %hi(__stack_top)
	la t0, isr
	csrw mtvec, t0
#if (VIRTUAL_MEMORY_ENABLE == 0)
	j	main
#else
	la a0, main
	j vm_boot
#endif

	.globl  pop_tf
pop_tf:
  lw  t0,33*4(a0)
  csrw  sepc,t0
  lw  x1,1*4(a0)
  lw  x2,2*4(a0)
  lw  x3,3*4(a0)
  lw  x4,4*4(a0)
  lw  x5,5*4(a0)
  lw  x6,6*4(a0)
  lw  x7,7*4(a0)
  lw  x8,8*4(a0)
  lw  x9,9*4(a0)
  lw  x11,11*4(a0)
  lw  x12,12*4(a0)
  lw  x13,13*4(a0)
  lw  x14,14*4(a0)
  lw  x15,15*4(a0)
  lw  x16,16*4(a0)
  lw  x17,17*4(a0)
  lw  x18,18*4(a0)
  lw  x19,19*4(a0)
  lw  x20,20*4(a0)
  lw  x21,21*4(a0)
  lw  x22,22*4(a0)
  lw  x23,23*4(a0)
  lw  x24,24*4(a0)
  lw  x25,25*4(a0)
  lw  x26,26*4(a0)
  lw  x27,27*4(a0)
  lw  x28,28*4(a0)
  lw  x29,29*4(a0)
  lw  x30,30*4(a0)
  lw  x31,31*4(a0)
  lw  a0,10*4(a0)
  sret
	