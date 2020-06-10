`timescale 1ns / 1ps
// =============================================================================
//  Program : memory_write_back.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Memory/Write_Back Pipeline Stage of the Aquila core (A RISC-V core).
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

module writeback #( parameter DATA_WIDTH = 32 )
(
    // External Signals
    input  wire                    clk_i,
    input  wire                    rst_i,

    // to RegisterFile
    output wire                    csr_we_o,
    output wire [11: 0]            csr_we_addr_o,
    output wire [DATA_WIDTH-1 : 0] csr_we_data_o,

    // to RegisterFile
    output wire                    rd_we_o,
    output wire [4 : 0]            rd_addr_o,
    output wire [DATA_WIDTH-1 : 0] rd_data_o,

    // System Jump operation
    input  wire                    sys_jump_i,
    input  wire [ 1: 0]            sys_jump_csr_addr_i,
    output wire                    sys_jump_o,
    output wire [ 1: 0]            sys_jump_csr_addr_o,

    //Exception From Memory
    input  wire                    exp_vld_i,
    input  wire                    exp_isinterrupt_i,
    input  wire [3 : 0]            exp_cause_i,
    input  wire [31: 0]            exp_tval_i,
    input  wire [31: 0]            instruction_pc_i,
    input  wire                    instruction_pc_vld_i,
        
    //To CSR     
    output wire                    exp_vld_o,
    output wire                    exp_isinterrupt_o,
    output wire [3 : 0]            exp_cause_o,
    output wire [31: 0]            exp_tval_o,
    output wire [31: 0]            instruction_pc_o,
    output wire                    instruction_pc_vld_o,
    output wire [31: 0]            mip_update_o,

    // from Execution Memory_Writeback_Pipeline
    input wire [2: 0]              regfile_input_sel_i,
    input  wire                    regfile_we_i,
    input  wire [4: 0]             rd_addr_i,
    input  wire                    mem_load_ext_sel_i,
    input  wire [1: 0]             mem_addr_alignment_i,
    input  wire [DATA_WIDTH-1 : 0] p_data_i,
    input  wire                    csr_we_i,
    input  wire [31: 0]            csr_we_addr_i,
    input  wire [31: 0]            csr_we_data_i,
    input  wire [DATA_WIDTH-1 : 0] mem_data_i,
    input  wire [31: 0]            mip_update_i
);

//=======================================================
// Parameter and Integer
//=======================================================

//=======================================================
// Wire and Reg 
//=======================================================
reg [31 : 0] aligned_data, rd_data;

//=======================================================
// User Logic                         
//=======================================================
always @(*)
begin
    case (mem_addr_alignment_i)
        2'b00:   aligned_data = mem_data_i;
        2'b01:   aligned_data = {mem_data_i[7: 0], mem_data_i[31: 8]};
        2'b10:   aligned_data = {mem_data_i[15: 0], mem_data_i[31: 16]};
        2'b11:   aligned_data = {mem_data_i[23: 0], mem_data_i[31: 24]};
        default: aligned_data = mem_data_i;
    endcase
end

always @(*)
begin
    case (regfile_input_sel_i)
        3'b000:  rd_data = mem_load_ext_sel_i ?
                          {24'b0, aligned_data[7 : 0]} :
                          {{24{aligned_data[7]}}, aligned_data[7 : 0]};   // load byte
        3'b001:  rd_data = mem_load_ext_sel_i ?
                          {16'b0, aligned_data[15 : 0]} :
                          {{16{aligned_data[15]}}, aligned_data[15 : 0]}; // load half word
        3'b010:  rd_data = aligned_data;                                   // load word
        default: rd_data = p_data_i;                                      // data from processor
    endcase
end

//=======================================================
// Output signals interface                       
//=======================================================
assign rd_we_o              = regfile_we_i;
assign rd_addr_o            = rd_addr_i;
assign rd_data_o            = rd_data;
assign sys_jump_o           = sys_jump_i;
assign sys_jump_csr_addr_o  = sys_jump_csr_addr_i;
assign csr_we_o             = csr_we_i;
assign csr_we_addr_o        = csr_we_addr_i;
assign csr_we_data_o        = csr_we_data_i;
assign exp_vld_o            = exp_vld_i;
assign exp_cause_o          = exp_cause_i;
assign exp_tval_o           = exp_tval_i;
assign instruction_pc_o     = instruction_pc_i;
assign instruction_pc_vld_o = instruction_pc_vld_i;

assign mip_update_o         = mip_update_i;
assign exp_isinterrupt_o    = exp_isinterrupt_i;

endmodule // memory_writeback

