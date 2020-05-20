`timescale 1ns / 1ps
// =============================================================================
//  Program : memory_access.v
//  Author  : Jin-you Wu
//  Date    : Dec/19/2018
// -----------------------------------------------------------------------------
//  Description:
//  This is the Memory Access Unit of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Nov/29/2019, by Chun-Jen Tsai:
//    Rename the old module 'memory_alignment' to 'memory_access.'
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

module memory_access #( parameter DATA_WIDTH = 32 )
(
    input  wire                     clk_i,
    input  wire                     rst_i,

    input  wire                     stall_i,
    output wire                     stall_for_data_fetch_o,
    input wire                      flush_i,

    // from Execute_Memory_Pipeline
    input  wire [DATA_WIDTH-1 : 0]  unaligned_data_i,       // from rs2
    input  wire [31: 0]             mem_addr_i,
    input  wire [1 : 0]             mem_input_sel_i,
    input  wire                     exe_we_i,
    input  wire                     exe_re_i,

    // to d-cache
    output reg  [DATA_WIDTH-1 : 0]   data_o,           // data_write
    output reg  [3: 0]               byte_write_sel_o,
    output wire                      req_vld_o,    
    output wire [DATA_WIDTH-1 : 0]   req_vaddr_o,       
    output wire                      req_rw_o,     

    input  wire                     d_rtrn_vld_i,
    input  wire [DATA_WIDTH-1 : 0]  d_rtrn_data_i,

    // System Jump operation
    input  wire                     sys_jump_i,
    input  wire [ 1: 0]             sys_jump_csr_addr_i,
    output wire                     sys_jump_o,
    output wire [ 1: 0]             sys_jump_csr_addr_o,

    // indicating memory alignment exception
    //output reg                      memory_alignment_exception,
    
    output wire                     req_done_o,        
    output wire                     buffer_exp_vld_o,  
    output wire [3 : 0]             buffer_exp_cause_o,
    output wire [31: 0]             buffer_exp_tval_o, 
    output wire [DATA_WIDTH-1 : 0]  buffer_data_o,    


    //exception from execute
    input  wire                     exp_vld_i,
    input  wire [3 : 0]             exp_cause_i,
    input  wire [31: 0]             exp_tval_i,
    input  wire [31: 0]             instruction_pc_i,

    //exception from mmu
    input  wire                     exp_from_mmu_vld_i,
    input  wire [3 : 0]             exp_from_mmu_cause_i,
    input  wire [31: 0]             exp_from_mmu_tval_i,

    //exception to writeback
    output reg                      exp_vld_o,
    output reg  [3 : 0]             exp_cause_o,
    output reg  [31: 0]             exp_tval_o,
    output wire [31: 0]             instruction_pc_o
);
localparam d_IDLE = 0, d_WAIT = 1;
reg dS, dS_nxt;
reg memory_alignment_exception;
assign stall_for_data_fetch_o = (dS_nxt == d_WAIT);

reg                     req_done;
reg                     buffer_exp_vld;
reg [3 : 0]             buffer_exp_cause;
reg [31: 0]             buffer_exp_tval;
reg [DATA_WIDTH-1 : 0]  buffer_data;

always @(posedge clk_i)
begin
    if (rst_i)
        dS <= d_IDLE;
    else
        dS <= dS_nxt;
end

always @(*)
begin
    case (dS)
        d_IDLE:
            if ((exe_re_i || exe_we_i) && !memory_alignment_exception && !flush_i && !req_done)
                dS_nxt = d_WAIT;
            else
                dS_nxt = d_IDLE;
        d_WAIT:
            if (d_rtrn_vld_i || exp_from_mmu_vld_i)
                dS_nxt = d_IDLE;
            else
                dS_nxt = d_WAIT;
    endcase
end

assign req_vld_o          = (dS_nxt == d_WAIT);      
assign req_vaddr_o        = mem_addr_i;
assign req_rw_o           = exe_we_i;

assign req_done_o           = req_done;         
assign buffer_exp_vld_o     = buffer_exp_vld;  
assign buffer_exp_cause_o   = buffer_exp_cause;
assign buffer_exp_tval_o    = buffer_exp_tval; 
assign buffer_data_o        = buffer_data;    


always@(posedge clk_i)
begin
    if(rst_i)
    begin
        req_done         <= 'd0;
        buffer_exp_vld   <= 'd0;
        buffer_exp_cause <= 'd0;
        buffer_exp_tval  <= 'd0;
        buffer_data      <= 'd0;
    end
    else if(dS == d_WAIT && (d_rtrn_vld_i || exp_from_mmu_vld_i) && stall_i && !stall_for_data_fetch_o)
    begin
        req_done         <= 'd1;
        buffer_exp_vld   <= exp_from_mmu_vld_i;
        buffer_exp_cause <= exp_from_mmu_cause_i;
        buffer_exp_tval  <= exp_from_mmu_tval_i;
        buffer_data      <= d_rtrn_data_i;
    end
    else if(stall_i)
    begin
        req_done         <= req_done;
        buffer_exp_vld   <= buffer_exp_vld;
        buffer_exp_cause <= buffer_exp_cause;
        buffer_exp_tval  <= buffer_exp_tval;
        buffer_data      <= buffer_data;
    end
    else
    begin
        req_done         <= 'd0;
        buffer_exp_vld   <= 'd0;
        buffer_exp_cause <= 'd0;
        buffer_exp_tval  <= 'd0;
        buffer_data      <= 'd0;
    end
end



assign sys_jump_o          = sys_jump_i;
assign sys_jump_csr_addr_o = sys_jump_csr_addr_i;
assign instruction_pc_o    = instruction_pc_i;

always@(*)
begin
    if(memory_alignment_exception && (exe_we_i || exe_re_i))
    begin
        exp_vld_o   = 1'b1;
        exp_cause_o = (exe_we_i)?'d6:'d4;
        exp_tval_o  = mem_addr_i;
    end
    else if(exp_from_mmu_vld_i)
    begin
        exp_vld_o   = 1'b1;
        exp_cause_o = exp_from_mmu_cause_i;
        exp_tval_o  = exp_from_mmu_tval_i;
    end
    else if(req_done)
    begin
        exp_vld_o   = buffer_exp_vld;
        exp_cause_o = buffer_exp_cause;
        exp_tval_o  = buffer_exp_tval;
    end
    else
    begin
        exp_vld_o   = exp_vld_i;
        exp_cause_o = exp_cause_i;
        exp_tval_o  = exp_tval_i;
    end
end

// store
always @(*)
begin
    case (mem_addr_i[1:0])
        2'b00:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin   // sb
                    data_o = {24'b0, unaligned_data_i[7: 0]};
                    byte_write_sel_o = 4'b0001;
                    memory_alignment_exception = 0;
                end
                2'b01:
                begin   // sh
                    data_o = {16'b0, unaligned_data_i[15: 0]};
                    byte_write_sel_o = 4'b0011;
                    memory_alignment_exception = 0;
                end
                2'b10:
                begin   // sw
                    data_o = unaligned_data_i;
                    byte_write_sel_o = 4'b1111;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data_o = 0;
                    byte_write_sel_o = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        2'b01:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin   // sb
                    data_o = {16'b0, unaligned_data_i[7: 0], 8'b0};
                    byte_write_sel_o = 4'b0010;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data_o = 0;
                    byte_write_sel_o = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        2'b10:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin
                    data_o = {8'b0, unaligned_data_i[7: 0], 16'b0};
                    byte_write_sel_o = 4'b0100;
                    memory_alignment_exception = 0;
                end
                2'b01:
                begin
                    data_o = {unaligned_data_i[15: 0], 16'b0};
                    byte_write_sel_o = 4'b1100;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data_o = 0;
                    byte_write_sel_o = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        2'b11:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin
                    data_o = {unaligned_data_i[7: 0], 24'b0};
                    byte_write_sel_o = 4'b1000;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data_o = 0;
                    byte_write_sel_o = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        default:
        begin
            data_o = 0;
            byte_write_sel_o = 4'b0000;
            memory_alignment_exception = 0;
        end
    endcase
end

endmodule   // memory_access
