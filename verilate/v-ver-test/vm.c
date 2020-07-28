#include "vm.h"
#include "encoding.h"
#include "handle_trap.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define pa2kva(pa) ((void*)(pa) - DRAM_BASE - MEGAPAGE_SIZE)

#define PAGE_NUM 512
pte_t pt[PAGE_NUM+1][PTES_PER_PT] __attribute__((aligned(RISCV_PGSIZE)));
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
  //user & kernal space
  for(int i = 0; i<PAGE_NUM; i++){
    pt[0][512+i] = ((pte_t)pt[i+1] >> RISCV_PGSHIFT << PTE_PPN_SHIFT) | PTE_V;
  }
  for(unsigned int i = 0; i < PAGE_NUM; i++) {
    for(unsigned int j = 0; j < 1024; ++j) {
      pt[i+1][j] = (unsigned int)((((unsigned int)(((unsigned int)DRAM_BASE)/RISCV_PGSIZE) + (i<<PTE_PPN_SHIFT) + j)) <<  PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D | PTE_U;
    }
  }
  //for uart
  //0xC0000000 ~ 0xC00FFFFF
  //0xC0000000 ~ 0xC00FFFFF
  pt[0][((((unsigned int)UART_BASE)>>PTE_PPN_SHIFT)>>RISCV_PGSHIFT)] = (((((unsigned int)UART_BASE))/RISCV_PGSIZE)<< PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D | PTE_U;

  //for client
  pt[0][((((unsigned int)CLIENT_BASE)>>PTE_PPN_SHIFT)>>RISCV_PGSHIFT)] = (((((unsigned int)CLIENT_BASE))/RISCV_PGSIZE)<< PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D | PTE_U;

  //for TCM
  pt[0][((((unsigned int)TCM_BASE)>>PTE_PPN_SHIFT)>>RISCV_PGSHIFT)] = (((((unsigned int)TCM_BASE))/RISCV_PGSIZE)<< PTE_PPN_SHIFT) | PTE_V | PTE_R | PTE_W | PTE_X | PTE_A | PTE_D | PTE_U;
  

  uintptr_t sptbr_value = (((uintptr_t)pt >> RISCV_PGSHIFT)| (0x80000000));
  write_csr(sptbr, sptbr_value);

  // set up supervisor trap handling
  write_csr(stvec, pa2kva(isr));
  write_csr(sscratch, pa2kva(read_csr(mscratch)));
  write_csr(medeleg,
    (1 << CAUSE_USER_ECALL) |
    (1 << CAUSE_FETCH_PAGE_FAULT) |
    (1 << CAUSE_LOAD_PAGE_FAULT) |
    (1 << CAUSE_STORE_PAGE_FAULT));
  // FPU on; accelerator on; allow supervisor access to user memory access
  set_csr(mstatus, MSTATUS_SUM);
  write_csr(mie, 0);
  write_csr(sepc, test_addr);
  
  printf("Virtual Memory boot done\n");
  
  asm volatile("lw	ra,60(sp)");
  asm volatile("addi	sp,sp,64");
  asm volatile("sret");
  // trapframe_t tf;
  // memset(&tf, 0, sizeof(tf));
  // tf.gpr[sp]
  // tf.epc = test_addr;
  // pop_tf(&tf);
}

