#include "vm.h"
#include "encoding.h"
#include "handle_trap.h"
#include <stdlib.h>
#include <string.h>

#define pa2kva(pa) ((void*)(pa) - DRAM_BASE - MEGAPAGE_SIZE)

#define PAGE_NUM 256
pte_t pt[PAGE_NUM+1][PTES_PER_PT];

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