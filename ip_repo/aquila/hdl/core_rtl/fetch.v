`timescale 1ns / 1ps
// =============================================================================
//  Program : fetch.v
//  Author  : Jin-you Wu
//  Date    : Dec/17/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the instruction Fetch stage of the Aquila core (A RISC-V core).
//
//  The pipeline architecture of Aquila 1.0 was based on the Microblaze-
//  compatible processor, KernelBlaze, designed by Dong-Fong Syu.
//  This file, fetch.v, was derived from fetch_decode.v of KernelBlaze by
//  Dong-Fong on Sep/01/2017.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Oct/17/2019, by Chun-Jen Tsai:
//    Change the initial program counter address from a "PARAMETER' to an
//    input signal, which come from a system register at the SoC-level.
//
//  Nov/28/2019, by Chun-Jen Tsai:
//    Parameterize DATA_WIDTH and rename the old module 'fetch_decode_pipeline'
//    to 'Fetch.'
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

module fetch #( parameter DATA_WIDTH = 32 )
(
    // External Signals
    input                         clk_i,
    input                         rst_i,

    // Singal to stall this Fetch_Decode_Pipeline Register
    input                         stall_i,

    // from Pipeline_Control
    input                         flush_i,

    // from i-memory
    input  [DATA_WIDTH-1 : 0]     instruction_i,

    // from Program_Counter
    input  [DATA_WIDTH-1 : 0]     pc_i,
    input  [DATA_WIDTH-1 : 0]     ppc_i,

    // from Cond_Branch_Predictor
    input                         cond_branch_hit_IF_i,
    input                         cond_branch_result_IF_i,

    // from Uncond_Branch_BHT
    input                         uncond_branch_hit_IF_i,

    // to the Decode Stage
    output reg [DATA_WIDTH-1 : 0] instruction_o,

    // to the Execute Stage
    output reg [DATA_WIDTH-1 : 0] pc_o,
    output reg                    instr_valid_o,
    output reg                    cond_branch_hit_ID_o,
    output reg                    cond_branch_result_ID_o,
    output reg                    uncond_branch_hit_ID_o,

    //Exception from MMU
    input  wire                    exp_from_mmu_vld_i,
    input  wire [ 3: 0]            exp_from_mmu_cause_i,
    input  wire [31: 0]            exp_from_mmu_tval_i,

    //Exception to Decode
    output reg                     exp_vld_o,
    output reg  [ 3: 0]            exp_cause_o,
    output reg  [31: 0]            exp_tval_o
);

reg [DATA_WIDTH-1 : 0] instruction_tcm;
reg [DATA_WIDTH-1 : 0] instruction_dram;

reg stall_delay;
reg flush_delay;
reg [31: 0] instruction_delay;

// **********
//  original
always @(posedge clk_i)
begin
    if (rst_i) begin // to-do: initialize value should be ?
        stall_delay <= 0;
        flush_delay <= 0;
        instruction_delay <= 32'h00000013;
    end else begin
        stall_delay <= stall_i;
        flush_delay <= flush_i;
        instruction_delay <= stall_delay ? instruction_delay : instruction_i;
    end
end

always @(*)
begin
    if (rst_i)
        instruction_tcm <= 32'h00000013;
    else if (stall_delay)
        instruction_tcm <= instruction_delay;
    else if (flush_delay)
        instruction_tcm <= 32'h00000013;
    else
        instruction_tcm <= instruction_i;
end

// *****************************

always @(posedge clk_i)
begin
    stall_delay <= stall_i;
    flush_delay <= flush_i;
    instruction_delay <= stall_i ? instruction_delay : instruction_i;
end

always @(*)
begin
    if (rst_i)
        instruction_tcm = 32'h00000013;
    else if (stall_delay)
        instruction_tcm = instruction_delay;
    else if (flush_delay)
        instruction_tcm = 32'h00000013;
    else
        instruction_tcm = instruction_delay;
end

always @(posedge clk_i)
begin
    if (rst_i)
        instruction_dram <= 32'h00000013;
    else if (stall_i)
        instruction_dram <= instruction_dram;
    else if (flush_i)
        instruction_dram <= 32'h00000013;
    else
        instruction_dram <= instruction_i;
end

always @(*) begin
    if(ppc_i[31:28] == 4'b0)
        instruction_o = instruction_tcm;
    else
        instruction_o = instruction_dram;
end

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        pc_o <= 32'h00000000;
        instr_valid_o <= 0;
        cond_branch_hit_ID_o <= 0;
        cond_branch_result_ID_o <= 0;
        uncond_branch_hit_ID_o <= 0;
        exp_vld_o <= 0;
        exp_cause_o <= 0;
        exp_tval_o <= 0;
    end
    else if (stall_i)
    begin
        pc_o <= pc_o;
        instr_valid_o <= instr_valid_o;
        cond_branch_hit_ID_o <= cond_branch_hit_ID_o;
        cond_branch_result_ID_o <= cond_branch_result_ID_o;
        uncond_branch_hit_ID_o <= uncond_branch_hit_ID_o;
        exp_vld_o <= exp_vld_o;
        exp_cause_o <= exp_cause_o;
        exp_tval_o <= exp_tval_o;
    end
    else if (flush_i)
    begin
        pc_o <= pc_i;
        instr_valid_o <= 0;
        cond_branch_hit_ID_o <= 0;
        cond_branch_result_ID_o <= 0;
        uncond_branch_hit_ID_o <= 0;
        exp_vld_o <= 0;
        exp_cause_o <= 0;
        exp_tval_o <= 0;
    end
    else if(exp_from_mmu_vld_i)
    begin
        pc_o <= pc_i;
        instr_valid_o <= 0; // default: instruction is valid
        cond_branch_hit_ID_o <= 0;
        cond_branch_result_ID_o <= 0;
        uncond_branch_hit_ID_o <= 0;
        exp_vld_o <= exp_from_mmu_vld_i;
        exp_cause_o <= exp_from_mmu_cause_i;
        exp_tval_o <= exp_from_mmu_tval_i;
    end
    else
    begin
        pc_o <= pc_i;
        instr_valid_o <= 1; // default: instruction is valid
        cond_branch_hit_ID_o <= cond_branch_hit_IF_i;
        cond_branch_result_ID_o <= cond_branch_result_IF_i;
        uncond_branch_hit_ID_o <= uncond_branch_hit_IF_i;
        exp_vld_o <= exp_from_mmu_vld_i;
        exp_cause_o <= exp_from_mmu_cause_i;
        exp_tval_o <= exp_from_mmu_tval_i;
    end
end

endmodule
