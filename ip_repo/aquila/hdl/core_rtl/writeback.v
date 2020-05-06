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

    // Signal that stalls this Memory-WriteBack Pipeline Stage
    input  wire                    stall_i,

    // Processor pipeline flush signal.
    input  wire                    flush_i,

    // from Execution Execute_Memory_Pipeline
    input  wire                    regfile_we_i,
    input  wire [4 : 0]            rd_addr_i,
    input  wire [2 : 0]            regfile_input_sel_i,
    input  wire                    mem_load_ext_sel_i,
    input  wire [1 : 0]            mem_addr_alignment_i,
    input  wire [DATA_WIDTH-1 : 0] p_data_i,
    input  wire                    csr_we_i,
    input  wire [11: 0]            csr_we_addr_i,
    input  wire [DATA_WIDTH-1 : 0] csr_we_data_i,

    // from Data Memory
    input  wire [DATA_WIDTH-1 : 0] mem_data_i,

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
    input  wire [3 : 0]            exp_cause_i,
    input  wire [31: 0]            exp_tval_i,
    input  wire [31: 0]            instruction_pc_i,
        
    //To CSR     
    output wire                    exp_vld_o,
    output wire [3 : 0]            exp_cause_o,
    output wire [31: 0]            exp_tval_o,
    output wire [31: 0]            instruction_pc_o
);

reg [2: 0]             regfile_input_sel_r;
reg                    regfile_we_r;
reg [4: 0]             rd_addr_r;
reg                    mem_load_ext_sel_r;
reg [1: 0]             mem_addr_alignment_r;
reg [DATA_WIDTH-1 : 0] p_data_r;

reg                    exp_vld_r;
reg [ 3: 0]            exp_cause_r;
reg [31: 0]            exp_tval_r;
reg                    sys_jump_r;
reg [ 1: 0]            sys_jump_csr_addr_r;
reg [31: 0]            instruction_pc_r;
reg                    csr_we_r;
reg [31: 0]            csr_we_addr_r;
reg [31: 0]            csr_we_data_r;

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        regfile_we_r         <= 0;
        rd_addr_r            <= 0;
        regfile_input_sel_r  <= 4;
        mem_load_ext_sel_r   <= 0;
        mem_addr_alignment_r <= 0;
        p_data_r             <= 0;
        csr_we_r             <= 0;
        csr_we_addr_r        <= 0;
        csr_we_data_r        <= 0;
        sys_jump_r           <= 0;
        sys_jump_csr_addr_r  <= 0;
        exp_vld_r            <= 0;
        exp_cause_r          <= 0;
        exp_tval_r           <= 0;
        instruction_pc_r     <= 0;
    end
    else if (stall_i)
    begin
        regfile_we_r         <= regfile_we_r;
        rd_addr_r            <= rd_addr_r;
        regfile_input_sel_r  <= regfile_input_sel_r;
        mem_load_ext_sel_r   <= mem_load_ext_sel_r;
        mem_addr_alignment_r <= mem_addr_alignment_r;
        p_data_r             <= p_data_r;
        csr_we_r             <= csr_we_r;
        csr_we_addr_r        <= csr_we_addr_r;
        csr_we_data_r        <= csr_we_data_r;
        sys_jump_r           <= sys_jump_r;
        sys_jump_csr_addr_r  <= sys_jump_csr_addr_r;
        exp_vld_r            <= exp_vld_r;
        exp_cause_r          <= exp_cause_r;
        exp_tval_r           <= exp_tval_r;
        instruction_pc_r     <= instruction_pc_r;
    end
    else if(flush_i)
    begin
        regfile_we_r         <= 0;
        rd_addr_r            <= 0;
        regfile_input_sel_r  <= 4;
        mem_load_ext_sel_r   <= 0;
        mem_addr_alignment_r <= 0;
        p_data_r             <= 0;
        csr_we_r             <= 0;
        csr_we_addr_r        <= 0;
        csr_we_data_r        <= 0;
        sys_jump_r           <= 0;
        sys_jump_csr_addr_r  <= 0;
        exp_vld_r            <= 0;
        exp_cause_r          <= 0;
        exp_tval_r           <= 0;
        instruction_pc_r     <= 0;
    end
    else if(exp_vld_i)
    begin
        regfile_we_r         <= 0;
        rd_addr_r            <= 0;
        regfile_input_sel_r  <= 4;
        mem_load_ext_sel_r   <= 0;
        mem_addr_alignment_r <= 0;
        p_data_r             <= 0;
        csr_we_r             <= 0;
        csr_we_addr_r        <= 0;
        csr_we_data_r        <= 0;
        sys_jump_r           <= 0;
        sys_jump_csr_addr_r  <= 0;
        exp_vld_r            <= exp_vld_i;
        exp_cause_r          <= exp_cause_i;
        exp_tval_r           <= exp_tval_i;
        instruction_pc_r     <= instruction_pc_i;
    end
    else
    begin
        regfile_we_r         <= regfile_we_i;
        rd_addr_r            <= rd_addr_i;
        regfile_input_sel_r  <= regfile_input_sel_i;
        mem_load_ext_sel_r   <= mem_load_ext_sel_i;
        mem_addr_alignment_r <= mem_addr_alignment_i;
        p_data_r             <= p_data_i;
        csr_we_r             <= csr_we_i;
        csr_we_addr_r        <= csr_we_addr_i;
        csr_we_data_r        <= csr_we_data_i;
        sys_jump_r           <= sys_jump_i;
        sys_jump_csr_addr_r  <= sys_jump_csr_addr_i;
        exp_vld_r            <= exp_vld_i;
        exp_cause_r          <= exp_cause_i;
        exp_tval_r           <= exp_tval_i;
        instruction_pc_r     <= instruction_pc_i;
    end
end

reg [31: 0] mem_data_r;
always @(posedge clk_i)
begin
    if (rst_i)
        mem_data_r <= 32'b0;
    else if (stall_i)
        mem_data_r <= mem_data_r;
    else if(exp_vld_i)
        mem_data_r <= 32'b0;
    else
        mem_data_r <= mem_data_i;
end

reg [31 : 0] aligned_data, rd_data;
always @(*)
begin
    case (mem_addr_alignment_r)
        2'b00:   aligned_data = mem_data_r;
        2'b01:   aligned_data = {mem_data_r[7: 0], mem_data_r[31: 8]};
        2'b10:   aligned_data = {mem_data_r[15: 0], mem_data_r[31: 16]};
        2'b11:   aligned_data = {mem_data_r[23: 0], mem_data_r[31: 24]};
        default: aligned_data = mem_data_r;
    endcase
end

always @(*)
begin
    case (regfile_input_sel_r)
        3'b000:  rd_data = mem_load_ext_sel_r ?
                          {24'b0, aligned_data[7 : 0]} :
                          {{24{aligned_data[7]}}, aligned_data[7 : 0]};   // load byte
        3'b001:  rd_data = mem_load_ext_sel_r ?
                          {16'b0, aligned_data[15 : 0]} :
                          {{16{aligned_data[15]}}, aligned_data[15 : 0]}; // load half word
        3'b010:  rd_data = aligned_data;                                   // load word
        default: rd_data = p_data_r;                                      // data from processor
    endcase
end

assign rd_we_o             = regfile_we_r;
assign rd_addr_o           = rd_addr_r;
assign rd_data_o           = rd_data;
assign sys_jump_o          = sys_jump_r;
assign sys_jump_csr_addr_o = sys_jump_csr_addr_r;
assign csr_we_o            = csr_we_r;
assign csr_we_addr_o       = csr_we_addr_r;
assign csr_we_data_o       = csr_we_data_r;
assign exp_vld_o           = exp_vld_r;
assign exp_cause_o         = exp_cause_r;
assign exp_tval_o          = exp_tval_r;
assign instruction_pc_o    = instruction_pc_r;

endmodule // memory_writeback

