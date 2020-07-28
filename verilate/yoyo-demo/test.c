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
#include "vm.h"
#include "uart.h"

clock_t         Begin_Time,
                End_Time,
                User_Time;

int main(void)
{
    printf("//-------------------------------------\n");
    printf("test one page load and store continuously\n");
    Begin_Time = clock();

    int x __attribute__((aligned(RISCV_PGSIZE)));
    for(int i = 0; i < RISCV_PGSIZE/4; ++i){
        *(&x+(i%4)) = i;
    }

    End_Time = clock();
    User_Time = End_Time - Begin_Time;
    printf("Time:%dus\n", User_Time);

    printf("\n");
    printf("//-------------------------------------\n");
    printf("test 256KB load and store continuously\n");
    Begin_Time = clock();

    for(int i = 0; i < RISCV_PGSIZE/4; ++i){
        *(&x+(i%8)) = i;
    }

    End_Time = clock();
    User_Time = End_Time - Begin_Time;
    printf("Time:%dus\n", User_Time);

    printf("\n");
    printf("//-------------------------------------\n");
    printf("test 256KB load and store discontinuous\n");
    Begin_Time = clock();

    for(int i = 0; i < RISCV_PGSIZE/4; ++i){
        *(&x+(i%16)) = i;
    }

    End_Time = clock();
    User_Time = End_Time - Begin_Time;
    printf("Time:%dus\n", User_Time);

    printf("\n");
    printf("//-------------------------------------\n");
    printf("test one page load and store continuously\n");
    Begin_Time = clock();

    for(int i = 0; i < RISCV_PGSIZE/4; ++i){
        *(&x+(i%(RISCV_PGSIZE*64/4)))) = i;
    }


    End_Time = clock();
    User_Time = End_Time - Begin_Time;
    printf("Time:%dus\n", User_Time);
    
    asm volatile ("ecall");
    printf("Test finaish.\n");
    return 0;
}