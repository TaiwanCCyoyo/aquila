`timescale 1ns / 1ps 
// =============================================================================
//  Program : program_counter.v
//  Author  : Jin-you Wu
//  Date    : Dec/17/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Unconditional Branch History Table of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Oct/17/2019, by Chun-Jen Tsai:
//    Change the initial program counter address from a "PARAMETER' to an
//    input signal, which come from a system register at the SoC-level.
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

module program_counter #( parameter ADDR_WIDTH = 32, DATA_WIDTH = 32 )
(
    // System signals
    input                     clk_i,
    input                     rst_i,

    // Program counter address at reset
    input  [ADDR_WIDTH-1 : 0] init_pc_addr_i,

    // Interrupt
    input                     irq_taken_i,
    input  [ADDR_WIDTH-1 : 0] PC_handler_i,

    // that stall Program_Counter
    input                     stall_i,

    // from Cond_Branch_Predictor
    input                     cond_branch_hit_IF_i,
    input                     cond_branch_result_IF_i,
    input  [ADDR_WIDTH-1 : 0] cond_branch_target_addr_i,

    // from Uncond_Branch_Predictor
    input                     uncond_branch_hit_IF_i,
    input  [ADDR_WIDTH-1 : 0] uncond_branch_target_addr_i,

    // System Jump operation
    input                     sys_jump_i,
    input  [ADDR_WIDTH-1 : 0] sys_jump_csr_data_i,

    // from Decode_Execute_Pipeline
    input                     cond_branch_hit_EXE_i,
    input                     cond_branch_result_EXE_i,
    input                     uncond_branch_hit_EXE_i,

    // from Execute
    input                     cond_branch_misprediction_i,
    input                     branch_taken_i,
    input  [ADDR_WIDTH-1 : 0] branch_target_addr_i,
    input  [ADDR_WIDTH-1 : 0] branch_restore_addr_i,  // already increment 4 in execute

    // to i-cache
    output [ADDR_WIDTH-1 : 0] pc_o,

    input wire         sfence_i,
    input wire [31: 0] dec_pc2exe_i
);

/* -------------------------------------------------------------------- *
 *      RISC-V has 32-bit instruction and 16-bit instruction(RVC)       *
 *      So, pc_r <= pc_r + 4   or   pc_r + 2;                           *
 *      However, we don't support rvc module currently                  *
 * -------------------------------------------------------------------- */
reg  [ADDR_WIDTH-1 : 0] pc_r;
wire                    rv32 = 1;
wire [ 2 : 0]           pc_offset = rv32 ? 4 : 2;
wire [ADDR_WIDTH-1 : 0] pc_increment;

assign pc_increment = pc_r + pc_offset;

reg irq_taken_hold;
reg [31: 0] PC_handler_hold;

always@(posedge clk_i)
begin
    if(rst_i)
    begin
        irq_taken_hold <= 0;
        PC_handler_hold <= 0;
    end
    else if (stall_i)
    begin
        if(!irq_taken_hold)
        begin
            irq_taken_hold <= irq_taken_i;
            PC_handler_hold <= PC_handler_i;
        end
        else
        begin
            irq_taken_hold <= irq_taken_hold;
            PC_handler_hold <= PC_handler_hold;
        end
    end
    else
    begin
        irq_taken_hold <= 0;
        PC_handler_hold <= 0;
    end
end

always @(posedge clk_i)
begin
    if (rst_i)
        pc_r <= init_pc_addr_i;
    else if (stall_i)
        pc_r <= pc_r;
    else if(irq_taken_hold)
        pc_r <= PC_handler_hold;
    else if (irq_taken_i)
        pc_r <= PC_handler_i;
    else if(sfence_i)
        pc_r <= dec_pc2exe_i;
    else
        // with branch predictor
        if (branch_taken_i & !uncond_branch_hit_EXE_i & !cond_branch_hit_EXE_i)
            pc_r <= branch_target_addr_i;
        else if (cond_branch_misprediction_i)
            pc_r <= cond_branch_result_EXE_i ? branch_restore_addr_i : branch_target_addr_i;
        else if (sys_jump_i)
            pc_r <= sys_jump_csr_data_i;
        else if (uncond_branch_hit_IF_i)
            pc_r <= uncond_branch_target_addr_i;
        else if (cond_branch_hit_IF_i & cond_branch_result_IF_i)
            pc_r <= cond_branch_target_addr_i;
        else
            pc_r <= pc_increment;
/*
    Supposedly, we can use the following code segment to replace the previous 
    else cases to disable branch-prediction. However, it did not work.

    if (branch_taken) pc_r <= branch_target_addr;
    else if (sys_jump) pc_r <= sys_jump_csr_data;
    else pc_r <= pc_increment;
*/
end

// ================================================================================
//  Output Signals
//
assign pc_o = pc_r;

endmodule   // program_counter
