#include "handle_trap.h"
#include "vm.h"
#include <stdio.h>
#include "uart.h"
//------------------------------
// ISR
void volatile isr()
{
    //printf("\nInterrupted or an exception occurred\n");
    //printf("Check what kind of Trap \n\n");
    asm volatile ("csrrs t0, mcause, zero");
    asm volatile ("srli t0, t0, 31");
    asm volatile ("beq	t0, zero, isr_is_exception");
    asm volatile ("j isr_is_interrupt");
}

void volatile isr_is_interrupt(){
    //printf("Interrupted\n");
    asm volatile ("csrrs t0, mcause, zero");
    asm volatile ("slli t0, t0, 1");
    asm volatile ("srli t0, t0, 1");
    asm volatile ("li	t1,0");
    asm volatile ("beq	t0, t1, isr_is_UserSoftwareInterrupt");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_SupervisorSoftwareInterrupt");
    asm volatile ("addi t1, t1, 2");
    asm volatile ("beq	t0, t1, isr_is_MachineSoftwareInterrupt");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_UserTimerInterrupt");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_SupervisorTimerInterrupt");
    asm volatile ("addi t1, t1, 2");
    asm volatile ("beq	t0, t1, isr_is_MachineTimerInterrupt");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_UserExternalInterrupt");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_SupaervisorExternalInterrupt");
    asm volatile ("addi t1, t1, 2");
    asm volatile ("beq	t0, t1, isr_is_MachineExternalInterrupt");
    asm volatile ("j isr_is_UnknownInterrupt");
}

void volatile isr_is_exception(){
    printf("Exception occurred\n");
    asm volatile ("csrrs t0, mcause, zero");
    asm volatile ("li	t1,0");
    asm volatile ("beq	t0, t1, isr_is_InstructionAddressMisaligned");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_InstructionAccessFault");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_IllegalInstruction");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_Breakpoint");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_LoadAddressMisaligned");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_LoadAccessFault");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_StoreAmoAddressMisaligned");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_StoreAmoAccessFault");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_EnvironmentCallFromUmode");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_EnvironmentCallFromSmode");
    asm volatile ("addi t1, t1, 2");
    asm volatile ("beq	t0, t1, isr_is_EnvironmentCallFromMmode");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_InstructionPageFault");
    asm volatile ("addi t1, t1, 1");
    asm volatile ("beq	t0, t1, isr_is_LoadPageFault");
    asm volatile ("addi t1, t1, 2");
    asm volatile ("beq	t0, t1, isr_is_StoreAmoPageFault");
    asm volatile ("j isr_is_UnknownException");
}

//------------------------------
// Interrupt
void volatile isr_is_UserSoftwareInterrupt(){
    printf("User software interrupt\n");
}
void volatile isr_is_SupervisorSoftwareInterrupt(){
    printf("Supervisor software interrupt\n");
}
void volatile isr_is_MachineSoftwareInterrupt(){
    printf("Machine software interrupt\n");
}
void volatile isr_is_UserTimerInterrupt(){
    printf("User timer interrupt\n");
}
void volatile isr_is_SupervisorTimerInterrupt(){
    printf("Supervisor timer interrupt\n");
}
void volatile isr_is_MachineTimerInterrupt(){
    printf("Machine timer interrupt\n");
    //mip should be clear by timer compare register.
    //wronge way to fix interrupt.
    asm volatile ("addi t0, zero, 128");
    asm volatile ("csrc mie, t0");
    got_timmer_isr = 1;
    asm volatile ("mret");
}
void volatile isr_is_UserExternalInterrupt(){
    printf("User external interrupt\n");
}
void volatile isr_is_SupaervisorExternalInterrupt(){
    printf("Supaervisor external interrupt\n");
}
void volatile isr_is_MachineExternalInterrupt(){
    printf("Machine external interrupt\n");
}
void volatile isr_is_UnknownInterrupt(){
    printf("Unknown interrupt\n");
}

//------------------------------
// Exception
void volatile isr_is_InstructionAddressMisaligned(){
    printf("Instruction address misaligned\n");
}
void volatile isr_is_InstructionAccessFault(){
    printf("Instruction access fault\n");
}
void volatile isr_is_IllegalInstruction(){
    printf("Illegal instruction\n");
}
void volatile isr_is_Breakpoint(){
    printf("isr_is_Breakpoint\n");
}
void volatile isr_is_LoadAddressMisaligned(){
    printf("Load address misaligned\n");
}
void volatile isr_is_LoadAccessFault(){
    printf("Load access fault\n");
}
void volatile isr_is_StoreAmoAddressMisaligned(){
    printf("Store\\Amo address misaligned\n");
}
void volatile isr_is_StoreAmoAccessFault(){
    printf("Store\\Amo access fault\n");
}
void volatile isr_is_EnvironmentCallFromUmode(){
    printf("Environment call from U-mode\n");
}
void volatile isr_is_EnvironmentCallFromSmode(){
    printf("Environment call from S-mode\n");
}
void volatile isr_is_EnvironmentCallFromMmode(){
    printf("Environment call from M-mode\n");
}
void volatile isr_is_InstructionPageFault(){
    printf("Instruction page fault\n");
}
void volatile isr_is_LoadPageFault(){
    printf("Load page fault\n");
}
void volatile isr_is_StoreAmoPageFault(){
    printf("Store\\Amo page fault\n");
}
void volatile isr_is_UnknownException(){
    printf("Unknown Exception\n");
}