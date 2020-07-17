`timescale 1 ns / 1 ps
// =============================================================================
//  Program : mmu.v
//  Author  : 
//  Date    : 
// -----------------------------------------------------------------------------
//  Description:
//  
// -----------------------------------------------------------------------------
//  Revision information:
//
//  
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

module mmu_arb #(
) (
    input  wire         clk_i,
    input  wire         rst_i,
    // input  wire         enable_i,
    input  wire         flush_i,

    //From processor to dcache
    input  wire         d_req_vld_i,         // request.
    input  wire [31: 0] d_req_paddr_i,       // Memory address.
    input  wire [31: 0] d_req_data_i,
    input  wire         d_req_rw_i,          // 0: data read, 1: data write.
    input  wire [ 3: 0] d_req_byte_enable_i,

    output wire         d_rtrn_vld_o,
    output wire [31: 0] d_rtrn_data_o,

    //From ptw to dcache
    input  wire         ptw_req_vld_i,         // request.
    input  wire [31: 0] ptw_req_paddr_i,       // Memory address.
    input  wire [31: 0] ptw_req_data_i,
    input  wire         ptw_req_rw_i,          // 0: data read, 1: data write.
    input  wire [ 3: 0] ptw_req_byte_enable_i,

    output wire         ptw_rtrn_vld_o,
    output wire [31: 0] ptw_rtrn_data_o,

    //To dcache
    output wire         d_req_vld_o,         // request.
    output wire         d_strobe_o,
    output wire [31: 0] d_req_paddr_o,       // Memory address.
    output wire [31: 0] d_req_data_o,
    output wire         d_req_rw_o,          // 0: data read, 1: data write.
    output wire [ 3: 0] d_req_byte_enable_o,

    input  wire         d_rtrn_vld_i,
    input  wire [31: 0] d_rtrn_data_i
);
    //=======================================================
    // Parameter and Integer
    //=======================================================

    //=======================================================
    // Wire and Reg 
    //=======================================================
    wire [ 1: 0] req_masked;
    wire [ 1: 0] mask_higher_pri_reqs;
    wire [ 1: 0] grant_masked;
    wire [ 1: 0] unmask_higher_pri_reqs;
    wire [ 1: 0] grant_unmasked;
    wire         no_req_masked;
    wire [ 1: 0] grant; // 1:ptw 0:d
    reg  [ 1: 0] pointer_reg, pointer_update_reg;

    reg          wait_for_dcache;

    //=======================================================
    // User Logic                         
    //=======================================================
    //-----------------------
    // Round Robin Arbiter Model

    // Simple priority arbitration for masked portion
    assign req_masked = {ptw_req_vld_i, d_req_vld_i} & pointer_reg;
    assign mask_higher_pri_reqs[1] = mask_higher_pri_reqs[0] | req_masked[0];
    assign mask_higher_pri_reqs[0] = 1'b0;
    assign grant_masked[ 1: 0] = req_masked[ 1: 0] & ~mask_higher_pri_reqs[ 1: 0];

    // Simple priority arbitration for unmasked portion
    assign unmask_higher_pri_reqs[1] = unmask_higher_pri_reqs[0] | d_req_vld_i;
    assign unmask_higher_pri_reqs[0] = 1'b0;
    assign grant_unmasked[ 1: 0] = {ptw_req_vld_i, d_req_vld_i} & ~unmask_higher_pri_reqs[ 1: 0];

    // Use grant_masked if there is any there, otherwise use grant_unmasked.
    assign no_req_masked = ~(|req_masked);
    assign grant = ({2{no_req_masked}} & grant_unmasked) | grant_masked;


    // Pointer update
    always @ (posedge clk_i) begin
        if (rst_i) begin
            pointer_update_reg <= 2'b11;
        end else begin
            // if( d_rtrn_vld_i ) begin
            if (|req_masked) begin // Which arbiter was used?
                pointer_update_reg <= mask_higher_pri_reqs;
            end else begin
                if (d_req_vld_i || ptw_req_vld_i) begin // Only update if there's a req
                    pointer_update_reg <= unmask_higher_pri_reqs;
                end else begin
                    pointer_update_reg <= pointer_update_reg ;
                end
            end
            // end
        end
    end

    always @ (posedge clk_i) begin
        if(rst_i) begin
            pointer_reg <= 2'b11;
        end else begin
            if(d_rtrn_vld_i) begin
                pointer_reg <= pointer_update_reg;
            end
        end 
    end

    reg [31: 0] req_paddr;
    always@(posedge clk_i) begin
        if(rst_i) begin
            req_paddr <= 0;
        end else if(grant[0]) begin
            req_paddr <= d_req_paddr_i;
        end else if(grant[1]) begin
            req_paddr <= ptw_req_paddr_i;
        end
    end 

    reg [31: 0] req_data;
    always@(posedge clk_i) begin
        if(rst_i) begin
            req_data <= 0;
        end else if(grant[0]) begin
            req_data <= d_req_data_i;
        end else if(grant[1]) begin
            req_data <= ptw_req_data_i;
        end
    end 

    reg req_rw;
    always@(posedge clk_i) begin
        if(rst_i) begin
            req_rw <= 0;
        end else if(grant[0]) begin
            req_rw <= d_req_rw_i;
        end else if(grant[1]) begin
            req_rw <= ptw_req_rw_i;
        end
    end 

    reg [3:0] req_byte_enable;
    always@(posedge clk_i) begin
        if(rst_i) begin
            req_byte_enable <= 0;
        end else if(grant[0]) begin
            req_byte_enable <= d_req_byte_enable_i;
        end else if(grant[1]) begin
            req_byte_enable <= ptw_req_byte_enable_i;
        end
    end 

    reg [1:0] req_vld;
    always@(posedge clk_i) begin
        if(rst_i) begin
            req_vld <= 2'b00;
        end else if(d_rtrn_vld_i) begin
            req_vld <= 2'b00;
        end else if(grant[0]) begin
            req_vld <= 2'b01;
        end else if(grant[1]) begin
            req_vld <= 2'b10;
        end else begin
            req_vld <= 2'b00;
        end
    end

    //=======================================================
    // Output signals interface                       
    //=======================================================
    assign d_req_vld_o         = (grant[0] & ~req_vld[0])         |  (grant[1] & ~ req_vld[1]);     
    assign d_req_paddr_o       = (grant[0])?d_req_paddr_i         :((grant[1])?ptw_req_paddr_i:req_paddr);  
    assign d_req_data_o        = (grant[0])?d_req_data_i          :((grant[1])?ptw_req_data_i:req_data);
    assign d_req_rw_o          = (grant[0])?d_req_rw_i            :((grant[1])?ptw_req_rw_i:req_rw);       
    assign d_req_byte_enable_o = (grant[0])?d_req_byte_enable_i   :((grant[1])?ptw_req_byte_enable_i:req_byte_enable);

    assign d_rtrn_vld_o        = pointer_update_reg[1]       & d_rtrn_vld_i;
    assign d_rtrn_data_o       = {32{pointer_update_reg[1]}} & d_rtrn_data_i;

    assign ptw_rtrn_vld_o      = ~pointer_update_reg[1]       & d_rtrn_vld_i;
    assign ptw_rtrn_data_o     = {32{~pointer_update_reg[1]}} & d_rtrn_data_i;


    // assign d_req_vld_o         = d_req_vld_i;
    // assign d_req_paddr_o       = d_req_paddr_i;  
    // assign d_req_data_o        = d_req_data_i;
    // assign d_req_rw_o          = d_req_rw_i;       
    // assign d_req_byte_enable_o = d_req_byte_enable_i;

    // assign d_rtrn_vld_o        =  d_rtrn_vld_i;
    // assign d_rtrn_data_o       =  d_rtrn_data_i;
endmodule