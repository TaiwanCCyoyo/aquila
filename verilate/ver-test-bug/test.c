// =============================================================================
//  Program : test.c
//  Author  : Chun-Jen Tsai
//  Date    : Dec/09/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the minimal time library for aquila.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "uart.h"

void malloc_test(int nwords);
void timer_isr_test();
void sleep(int msec);

volatile int got_timmer_isr;

//------------------------------
// ISR
void volatile isr()
{
    printf("\nInterrupted or an exception occurred\n");
    printf("Check what kind of Trap \n\n");
    asm volatile ("addi t0, zero, 0");
    asm volatile ("csrc mcause, t0");
    asm volatile ("srli t0, t0, 31");
    asm volatile ("beq	t0, zero, isr_is_interrupt");
    asm volatile ("j isr_is_exception");
}

void volatile isr_is_interrupt(){
    printf("Interrupted\n");
    asm volatile ("addi t0, zero, 0");
    asm volatile ("csrc mcause, t0");
    asm volatile ("slli t0, t0, 1");
    asm volatile ("srli t0, t0, 1");
    asm volatile ("addi t1, zero, 0");
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
    asm volatile ("addi t0, zero, 0");
    asm volatile ("csrc mcause, t0");
    asm volatile ("addi t1, zero, 0");
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
//------------------------------
// 
void volatile install_isr(unsigned int isr)
{
    // the parameter is stored in the a0 register.
    // asm volatile ("addi t0, a0, 0x0");
    // asm volatile ("addi	t0,t0,0x1");
	// asm volatile ("csrw mtvec, t0");
    asm volatile ("csrw mtvec, a0");
	printf("Installed ISR at 0x%x\n", isr);
}

void volatile set_timer_period(unsigned long msec)
{
    unsigned long volatile *clint_mem = (unsigned long *) 0xF0000000;
    clint_mem[3] = 0;
    clint_mem[2] = msec;
    clint_mem[0] = clint_mem[1] = 0;
}

void volatile enable_core_timer()
{
    asm volatile ("addi t0, zero, 128");
    asm volatile ("csrw mie, t0");
}

int main(void)
{
    outbyte('H');
    outbyte('e');
    outbyte('l');
    outbyte('l');
    outbyte('o');
    outbyte(' ');
    outbyte('W');
    outbyte('o');
    outbyte('r');
    outbyte('l');
    outbyte('d');
    outbyte('!');
    outbyte('\n');

    printf("Test Print\n");
    float ver = 0.9;
    printf("Hello world!\n");
    printf("Hello, Aquila %.1f!\n", ver);
    printf("The address of 'ver' is 0x%X\n\n", (unsigned) &ver);

    printf("First time tick = %d\n\n", clock());
    malloc_test(24); // malloc return bug
    printf("\nSecond time tick = %d\n\n", clock());

    timer_isr_test();
    printf("Waiting for timer ISR ...\n");

    
    while (! got_timmer_isr)
    {
        /* busy waiting */
    }
    printf("Test finished.\n");
    return 0;
}

void malloc_test(int nwords)
{
    int *buf, idx;

    printf("Memory allocation test of %d words:\n", nwords);
    if ((buf = (int *) malloc(nwords*4)) == NULL)
    {
        printf("Error: Out of memory.\n");
        exit(-1);
    }
    printf("The buffer address is: 0x%X\n", (unsigned) buf);
    for (idx = 0; idx < nwords; idx++) buf[idx] = idx;
    for (idx = 0; idx < 10; idx++)
    {
        printf("Addr 0x%X, buf[%d] = %d\n", (unsigned) &(buf[idx]), idx, buf[idx]);
    }
    printf("\n...\n");
    for (idx = 10; idx > 0; idx--)
    {
        printf("Addr 0x%X, buf[%d] = %d\n",(unsigned) &(buf[idx]), nwords-idx, buf[nwords-idx]);
    }
    free(buf);
    printf("Buffer freed.\n");
}

void timer_isr_test()
{
    // char str[10];
    int n;

    printf("Timer ISR test:1\n");
    n = 3;

    // Set the ISR address.
    got_timmer_isr = 0;
    install_isr((unsigned int) isr);
    printf("install_isr done\n");

    // Input the timer interrupt duration.
    // do
    // {
    //     printf("Input the interrupt duration (in msec): ");
    //     fgets(str, sizeof(str), stdin);
    //     n = atoi(str);
    // } while (n == 0);

    // Set the interrupt duration.
    set_timer_period(n);
    printf("set_timer_period done\n");

    // Enable the timer interrupts.
    enable_core_timer();
    printf("enable_core_timer done\n");
}
