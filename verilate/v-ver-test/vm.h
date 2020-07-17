#ifndef VM_H_
#define VM_H_

#define VIRTUAL_MEMORY_ENABLE 1
# define RISCV_PGLEVEL_BITS 10
#define RISCV_PGSHIFT 12
#define PTE_PPN_SHIFT 10
#define RISCV_PGSIZE (1 << RISCV_PGSHIFT)

#define PTES_PER_PT (1 << RISCV_PGLEVEL_BITS)
#define MEGAPAGE_SIZE (PTES_PER_PT * RISCV_PGSIZE)

#define pa2kva(pa) ((void*)(pa) - DRAM_BASE - MEGAPAGE_SIZE)

typedef unsigned long pte_t;
typedef unsigned int uintptr_t;

typedef struct
{
  long gpr[32];
  long sr;
  long epc;
  long badvaddr;
  long cause;
} trapframe_t;

void vm_boot(uintptr_t test_addr);



#endif