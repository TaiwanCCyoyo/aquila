#include "vm.h"
#include "encoding.h"
#include "handle_trap.h"
#include <stdlib.h>
#include <string.h>

#define pa2kva(pa) ((void*)(pa) - DRAM_BASE - MEGAPAGE_SIZE)

#define PAGE_NUM 256
pte_t pt[PAGE_NUM+1][PTES_PER_PT];

void pop_tf(trapframe_t* tf_ptr){
  asm volatile("lw  t0,33*4(a0)");
  asm volatile("csrw  sepc,t0");
  asm volatile("lw  x1,1*4(a0)");
  asm volatile("lw  x2,2*4(a0)");
  asm volatile("lw  x3,3*4(a0)");
  asm volatile("lw  x4,4*4(a0)");
  asm volatile("lw  x5,5*4(a0)");
  asm volatile("lw  x6,6*4(a0)");
  asm volatile("lw  x7,7*4(a0)");
  asm volatile("lw  x8,8*4(a0)");
  asm volatile("lw  x9,9*4(a0)");
  asm volatile("lw  x11,11*4(a0)");
  asm volatile("lw  x12,12*4(a0)");
  asm volatile("lw  x13,13*4(a0)");
  asm volatile("lw  x14,14*4(a0)");
  asm volatile("lw  x15,15*4(a0)");
  asm volatile("lw  x16,16*4(a0)");
  asm volatile("lw  x17,17*4(a0)");
  asm volatile("lw  x18,18*4(a0)");
  asm volatile("lw  x19,19*4(a0)");
  asm volatile("lw  x20,20*4(a0)");
  asm volatile("lw  x21,21*4(a0)");
  asm volatile("lw  x22,22*4(a0)");
  asm volatile("lw  x23,23*4(a0)");
  asm volatile("lw  x24,24*4(a0)");
  asm volatile("lw  x25,25*4(a0)");
  asm volatile("lw  x26,26*4(a0)");
  asm volatile("lw  x27,27*4(a0)");
  asm volatile("lw  x28,28*4(a0)");
  asm volatile("lw  x29,29*4(a0)");
  asm volatile("lw  x30,30*4(a0)");
  asm volatile("lw  x31,31*4(a0)");
  asm volatile("lw  a0,10*4(a0)");
  asm volatile("sret");
}

void vm_boot(uintptr_t test_addr)
{
  // map user to lowermost megapage
  //0x80000000 ~ 0xBFFFFFFF
  //0x00000000 ~ 0x3FFFFFFF
  //0x00000000 ~ 0x003FFFFF 4KB
  //0x00400000 ~ 0x006FFFFF
  //...
  //0x3FC00000 ~ 0x3FFFFFFF
  //共256頁
  //=> user space
  for(int i = 0; i<PAGE_NUM; i++){
    pt[0][i] = ((pte_t)pt[i+i] >> RISCV_PGSHIFT << PTE_PPN_SHIFT) | PTE_V;
  }
  
  // map kernel to uppermost megapage
  //0xFFF00000 ~ 0xFFFFFFFFF
  //=> kernal space
  //
  pt[0][PTES_PER_PT-1] = (DRAM_BASE/RISCV_PGSIZE << PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;

  //for uart
  //0xC0000000 ~ 0xC00FFFFF
  //0xC0000000 ~ 0xC00FFFFF
  pt[0][((((unsigned int)UART_BASE)>>PTE_PPN_SHIFT)>>RISCV_PGSHIFT)] = (((((unsigned int)UART_BASE))/RISCV_PGSIZE)<< PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;

  // set up supervisor trap handling
  write_csr(stvec, pa2kva(isr));
  write_csr(sscratch, pa2kva(read_csr(mscratch)));
  write_csr(medeleg,
    (1 << CAUSE_USER_ECALL) |
    (1 << CAUSE_FETCH_PAGE_FAULT) |
    (1 << CAUSE_LOAD_PAGE_FAULT) |
    (1 << CAUSE_STORE_PAGE_FAULT));
  // FPU on; accelerator on; allow supervisor access to user memory access
  write_csr(mstatus, MSTATUS_SUM);
  write_csr(mie, 0);

  for(int i = 0; i < PAGE_NUM; i++) {
    for(int j = 0; j < 1024; ++j) {
      pt[1+i][j] = ((((unsigned int)DRAM_BASE)/RISCV_PGSIZE + i) <<  PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D;
    }
  }

  trapframe_t tf;
  memset(&tf, 0, sizeof(tf));
  tf.epc = test_addr - DRAM_BASE;
  pop_tf(&tf);
}

