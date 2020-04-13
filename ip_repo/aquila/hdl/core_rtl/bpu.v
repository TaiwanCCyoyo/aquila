`timescale 1ns / 1ps 
// =============================================================================
//  Program : bpu.v
//  Author  : Jin-you Wu
//  Date    : Jan/19/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the Branch prediction unit of the Aquila core (A RISC-V core).
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

module bpu #( parameter ENTRY_NUM = 24, ADDR_WIDTH = 5, DATA_WIDTH = 32 )
(
    // System signals
    input                       clk_i,
    input                       rst_i,
    input                       stall_i,

    // from Program_Counter
    input  [DATA_WIDTH - 1 : 0] pc_IF_i,

    // from Decode_Execute_Pipeline
    input                       is_cond_branch_i,
    input  [DATA_WIDTH - 1 : 0] pc_EXE_i,

    // from Execute
    input                       branch_taken_i,
    input  [DATA_WIDTH-1 : 0]   branch_target_addr_i,
    input                       cond_branch_misprediction_i,

    // to Program_Counter
    output                      cond_branch_hit_o,
    output                      cond_branch_result_o,
    output [DATA_WIDTH-1 : 0]   cond_branch_target_addr_o
);

// ===========================================================================
//
wire we;
wire [DATA_WIDTH - 1 : 0] data_out;
wire [ENTRY_NUM - 1  : 0] addr_hit_IF, addr_hit_EXE;
reg  [ADDR_WIDTH - 1 : 0] read_addr, update_addr;
reg  [ADDR_WIDTH - 1 : 0] write_addr;
reg  [DATA_WIDTH - 1 : 0] cond_branch_pc_table[ENTRY_NUM - 1 : 0];

// two-bit saturating counter
reg  [1: 0]               prediction_table[ENTRY_NUM - 1 : 0];

assign we = ~stall_i & is_cond_branch_i & ~|addr_hit_EXE; // CY Hsiang 0220_2020 15:50

genvar i;
generate
    for (i = 0; i < ENTRY_NUM; i = i + 1)
    begin
        assign addr_hit_IF[i] = (cond_branch_pc_table[i] == pc_IF_i);
        assign addr_hit_EXE[i] = (cond_branch_pc_table[i] == pc_EXE_i);
    end
endgenerate

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        write_addr <= 0;
    end
    else if (stall_i)
    begin
        write_addr <= write_addr;
    end
    else if (we)
    begin
        write_addr <= (write_addr == (ENTRY_NUM - 1)) ? 0 : write_addr + 1;
    end
end

/* ******************************************************************** *
 *  Mapping example of read_addr/update_addr:                           *
 *      addr_hit_XX     value   ->  the corresponding address           *
 *      000001           1      ->  0                                   *
 *      000010           2      ->  1                                   *
 *      000100           4      ->  2                                   *
 *      001000           8      ->  3                                   *
 *      010000          16      ->  4                                   *
 *      100000          32      ->  5                                   *
 *  So, we can get the read_addr from log2 function                     *
 * ******************************************************************** */ 
// assign read_addr = clogb2(addr_hit_IF);
// assign update_addr = clogb2(addr_hit_EXE);
always @( * )
begin
    case (addr_hit_IF)
        32'h0000_0002: read_addr <= 1 ;
        32'h0000_0004: read_addr <= 2 ;
        32'h0000_0008: read_addr <= 3 ;
        32'h0000_0010: read_addr <= 4 ;
        32'h0000_0020: read_addr <= 5 ;
        32'h0000_0040: read_addr <= 6 ;
        32'h0000_0080: read_addr <= 7 ;
        32'h0000_0100: read_addr <= 8 ;
        32'h0000_0200: read_addr <= 9 ;
        32'h0000_0400: read_addr <= 10;
        32'h0000_0800: read_addr <= 11;
        32'h0000_1000: read_addr <= 12;
        32'h0000_2000: read_addr <= 13;
        32'h0000_4000: read_addr <= 14;
        32'h0000_8000: read_addr <= 15;
        32'h0001_0000: read_addr <= 16;
        32'h0002_0000: read_addr <= 17;
        32'h0004_0000: read_addr <= 18;
        32'h0008_0000: read_addr <= 19;
        32'h0010_0000: read_addr <= 20;
        32'h0020_0000: read_addr <= 21;
        32'h0040_0000: read_addr <= 22;
        32'h0080_0000: read_addr <= 23;
        32'h0100_0000: read_addr <= 24;
        32'h0200_0000: read_addr <= 25;
        32'h0400_0000: read_addr <= 26;
        32'h0800_0000: read_addr <= 27;
        32'h1000_0000: read_addr <= 28;
        32'h2000_0000: read_addr <= 29;
        32'h4000_0000: read_addr <= 30;
        32'h8000_0000: read_addr <= 31;
        default:       read_addr <= 0;  //32'h0000_0001
    endcase
end
always @( * )
begin
    case (addr_hit_EXE)
        32'h0000_0002: update_addr <= 1 ;
        32'h0000_0004: update_addr <= 2 ;
        32'h0000_0008: update_addr <= 3 ;
        32'h0000_0010: update_addr <= 4 ;
        32'h0000_0020: update_addr <= 5 ;
        32'h0000_0040: update_addr <= 6 ;
        32'h0000_0080: update_addr <= 7 ;
        32'h0000_0100: update_addr <= 8 ;
        32'h0000_0200: update_addr <= 9 ;
        32'h0000_0400: update_addr <= 10;
        32'h0000_0800: update_addr <= 11;
        32'h0000_1000: update_addr <= 12;
        32'h0000_2000: update_addr <= 13;
        32'h0000_4000: update_addr <= 14;
        32'h0000_8000: update_addr <= 15;
        32'h0001_0000: update_addr <= 16;
        32'h0002_0000: update_addr <= 17;
        32'h0004_0000: update_addr <= 18;
        32'h0008_0000: update_addr <= 19;
        32'h0010_0000: update_addr <= 20;
        32'h0020_0000: update_addr <= 21;
        32'h0040_0000: update_addr <= 22;
        32'h0080_0000: update_addr <= 23;
        32'h0100_0000: update_addr <= 24;
        32'h0200_0000: update_addr <= 25;
        32'h0400_0000: update_addr <= 26;
        32'h0800_0000: update_addr <= 27;
        32'h1000_0000: update_addr <= 28;
        32'h2000_0000: update_addr <= 29;
        32'h4000_0000: update_addr <= 30;
        32'h8000_0000: update_addr <= 31;
        default:       update_addr <= 0;  //32'h0000_0001
    endcase
end

integer idx;
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
            cond_branch_pc_table[idx] <= 0;
    end
    else if (stall_i)
    begin
        for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
            cond_branch_pc_table[idx] <= cond_branch_pc_table[idx];
    end
    else if (we)
    begin
        cond_branch_pc_table[write_addr] <= pc_EXE_i;
    end
end

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
            prediction_table[idx] <= 2'b0;
    end
    else if (stall_i)
    begin
        for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
            prediction_table[idx] <= prediction_table[idx];
    end
    else
    begin
        if (we)
        begin
            prediction_table[write_addr] <= {branch_taken_i, branch_taken_i};
        end
        else if (cond_branch_misprediction_i)
        begin
            case (prediction_table[update_addr])
                2'b00:  // strongly not taken
                    if (branch_taken_i)
                        prediction_table[update_addr] <= 2'b01;
                    else
                        prediction_table[update_addr] <= 2'b00;
                2'b01:  // weakly not taken
                    if (branch_taken_i)
                        prediction_table[update_addr] <= 2'b11;
                    else
                        prediction_table[update_addr] <= 2'b00;
                2'b10:  // weakly taken
                    if (branch_taken_i)
                        prediction_table[update_addr] <= 2'b11;
                    else
                        prediction_table[update_addr] <= 2'b00;
                2'b11:  // strongly taken
                    if (branch_taken_i)
                        prediction_table[update_addr] <= 2'b11;
                    else
                        prediction_table[update_addr] <= 2'b10;
            endcase
        end
    end
end

// ===========================================================================
//  Submodule
//
distri_ram #(.ENTRY_NUM(ENTRY_NUM), .ADDR_WIDTH(ADDR_WIDTH), .DATA_WIDTH(DATA_WIDTH))
           BPU_distri_ram_0(
               .clk_i(clk_i),
               .we_i(we),
               .write_addr_i(write_addr),
               .read_addr_i(read_addr),
               .branch_target_addr_i(branch_target_addr_i),
               .data_o(data_out)
           );

// ===========================================================================
//  Outputs signals
//
assign cond_branch_hit_o = ( | addr_hit_IF) & ( | pc_IF_i);
assign cond_branch_result_o = ( | prediction_table[read_addr][1]);
assign cond_branch_target_addr_o = {32{( | addr_hit_IF)}} & data_out;

endmodule   // bpu
