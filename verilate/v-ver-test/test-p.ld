/*
// =============================================================================
//  Program : test.ld
//  Author  : Chun-Jen Tsai
//  Date    : Dec/09/2019
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  Description:
//  This is the boot code for Aquila SoC.  Upon reset, the boot code waiting
//  for a program to be transferred from the UART port (using [File] -> [Send file]
//  menu command of TeraTerm). Then the program will be loaded to 0x9000_0000,
//  and executed. The processor will be halted when the execution is finished.
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
must start from 0x80000000 due to aquila hard code /
*/

__heap_start = 0x80220000;
__heap_size  = 0x00001000;
__stack_top  = 0x80230000;
__stack_size = 0x00001000;
__stack_start = __stack_top - __stack_size;

MEMORY
{
    code_ram   (rx!rw) : ORIGIN = 0x80000000, LENGTH = 0x00008000
    data_ram   (rw!x)  : ORIGIN = 0x8000A000, LENGTH = 0x00210000
    heap_ram   (rw!x)  : ORIGIN = __heap_start, LENGTH = __heap_size
    stack_ram  (rw!x)  : ORIGIN = __stack_start, LENGTH = __stack_size
    IO       (rw!x)  : ORIGIN = 0xC0000000, LENGTH = 0x00000010
    TIMER    (rw!x)  : ORIGIN = 0xF0000000, LENGTH = 0x00000020
}
SECTIONS
{
    .text :
    {
        boot-p.o(.text)
        *(.text)

    } > code_ram

    .data :
    {
        *(.data)
        *(.bss)
        *(.rodata*)

    } > data_ram
}
