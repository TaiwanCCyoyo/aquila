`timescale 1ns / 1ps
// =============================================================================
//  Program : forwarding_unit.v
//  Author  : Jin-you Wu
//  Date    : Dec/17/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Data Forwarding Unit of the Aquila core (A RISC-V core).
//
//  The Data Forwarding Unit was based on the Microblaze-compatible processor,
//  KernelBlaze, originally designed by Dong-Fong Syu on Sep/01/2017.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  NONE.
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

module forwarding_unit #( parameter DATA_WIDTH = 32 )
(
    // from Decode_Eexcute_Pipeline Register
    input wire  [ 4: 0]            rs1_addr_i,
    input wire  [ 4: 0]            rs2_addr_i,
    input wire  [11: 0]            csr_addr_i,
    input wire  [DATA_WIDTH-1 : 0] rs1_data_i,
    input wire  [DATA_WIDTH-1 : 0] rs2_data_i,
    input wire  [DATA_WIDTH-1 : 0] csr_data_i,

    // from Execute_Memory_Pipeline Register
    input wire  [4: 0]             rd_addr_EXE_MEM_i,
    input wire                     regfile_we_EXE_MEM_i,
    input wire  [2: 0]             regfile_input_sel_EXE_MEM_i,
    input wire  [DATA_WIDTH-1 : 0] p_data_EXE_MEM_i,
    input wire  [11: 0]            csr_addr_EXE_MEM_i,
    input wire                     csr_we_EXE_MEM_i,
    input wire  [DATA_WIDTH-1 : 0] csr_data_EXE_MEM_i,

    // from Memory_Write_Back_Pipeline Register
    input wire  [4: 0]             rd_addr_MEM_WB_i,
    input wire                     regfile_we_MEM_WB_i,
    input wire  [DATA_WIDTH-1 : 0] rd_data_MEM_WB_i,
    input wire  [11: 0]            csr_addr_MEM_WB_i,
    input wire                     csr_we_MEM_WB_i,
    input wire  [DATA_WIDTH-1 : 0] csr_data_MEM_WB_i,

    // to Execution Stage
    output wire [DATA_WIDTH-1 : 0] rs1_fwd_o,
    output wire [DATA_WIDTH-1 : 0] rs2_fwd_o,
    output wire [DATA_WIDTH-1 : 0] csr_fwd_o
);

wire is_rs1_rd_EXE_MEM_same, is_rs2_rd_EXE_MEM_same, is_csr_addr_EXE_MEM_same;
wire is_rs1_rd_MEM_WB_same, is_rs2_rd_MEM_WB_same, is_csr_addr_MEM_WB_same;
wire is_rd_EXE_MEM_not_zero, is_rd_MEM_WB_not_zero;

wire rs1_EXE_MEM_fwd, rs2_EXE_MEM_fwd, csr_EXE_MEM_fwd;
wire rs1_MEM_WB_fwd, rs2_MEM_WB_fwd, csr_MEM_WB_fwd;

wire [DATA_WIDTH-1 : 0] correct_rs1_data, correct_rs2_data, correct_csr_data;

wire [DATA_WIDTH-1 : 0] correct_fwd_src2;
wire [DATA_WIDTH-1 : 0] correct_fwd_src1;

assign is_rs1_rd_EXE_MEM_same = (rs1_addr_i == rd_addr_EXE_MEM_i);
assign is_rs2_rd_EXE_MEM_same = (rs2_addr_i == rd_addr_EXE_MEM_i);
assign is_csr_addr_EXE_MEM_same = (csr_addr_i == csr_addr_EXE_MEM_i);
assign is_rs1_rd_MEM_WB_same = (rs1_addr_i == rd_addr_MEM_WB_i);
assign is_rs2_rd_MEM_WB_same = (rs2_addr_i == rd_addr_MEM_WB_i);
assign is_csr_addr_MEM_WB_same = (csr_addr_i == csr_addr_MEM_WB_i);

assign is_rd_EXE_MEM_not_zero = ( | rd_addr_EXE_MEM_i);
assign is_rd_MEM_WB_not_zero = ( | rd_addr_MEM_WB_i);

assign rs1_EXE_MEM_fwd = regfile_we_EXE_MEM_i & is_rs1_rd_EXE_MEM_same & is_rd_EXE_MEM_not_zero;
assign rs2_EXE_MEM_fwd = regfile_we_EXE_MEM_i & is_rs2_rd_EXE_MEM_same & is_rd_EXE_MEM_not_zero;
assign csr_EXE_MEM_fwd = csr_we_EXE_MEM_i & is_csr_addr_EXE_MEM_same;
assign rs1_MEM_WB_fwd = regfile_we_MEM_WB_i & is_rs1_rd_MEM_WB_same & is_rd_MEM_WB_not_zero;
assign rs2_MEM_WB_fwd = regfile_we_MEM_WB_i & is_rs2_rd_MEM_WB_same & is_rd_MEM_WB_not_zero;
assign csr_MEM_WB_fwd = csr_we_MEM_WB_i & is_csr_addr_MEM_WB_same;

assign correct_fwd_src2 = rd_data_MEM_WB_i;
assign correct_fwd_src1 = p_data_EXE_MEM_i;    // 'd0, 'd1, 'd2 case will handle by
                                               // pipeline_control (load-use data hazard)

assign correct_rs1_data =
       rs1_EXE_MEM_fwd ? correct_fwd_src1
       : rs1_MEM_WB_fwd ? correct_fwd_src2
       : rs1_data_i;

assign correct_rs2_data =
       rs2_EXE_MEM_fwd ? correct_fwd_src1
       : rs2_MEM_WB_fwd ? correct_fwd_src2
       : rs2_data_i;

assign correct_csr_data =
       csr_EXE_MEM_fwd ? csr_data_EXE_MEM_i
       : csr_MEM_WB_fwd ? csr_data_MEM_WB_i
       : csr_data_i;


// ================================================================================
//  Outputs signals
//
assign rs1_fwd_o = correct_rs1_data;
assign rs2_fwd_o = correct_rs2_data;
assign csr_fwd_o = correct_csr_data;

endmodule   // forwarding
