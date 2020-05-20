`timescale 1 ns / 1 ps
// =============================================================================
//  Program : tlb.v
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

module tlb # (
    parameter TLB_ENTRIES = 4
)(
    input wire          clk_i,
    input wire          rst_i,

    // Flush
    input wire          flush_i,

    // Enable
    input wire          enable_i,

    // Update TLB
    input wire          update_vld_i,
    input wire [19: 0]  update_tag_i,
    input wire [31: 0]  update_content_i,
    input wire          update_content_is_4MB_i,

    // Translate addres
    input  wire [ 8: 0] asid_i,
    input  wire [31: 0] vaddr_i,
    
    output wire         hit_o,
    output wire [31: 0] paddr_o,

    output wire [31: 0] content_o
);
//=======================================================
// Parameter and Integer
//=======================================================
integer i;
genvar  g;
//=======================================================
// Wire and Reg 
//=======================================================
//-----------------------------------------------
// PTE page table entry
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
// | ppn_1 | ppn_0 | rsw | D(dirty) | A(accessed) | G(global) | U(user) | X(executable) | W(writable) | R(readable) | V(valid) |
// -----------------------------------------------------------------------------------------------------------------------------
// |31   20|19   10|9   8|    7     |      6      |     5     |    4    |       3       |      2      |      1      |     0    |
// -----------------------------------------------------------------------------------------------------------------------------
reg  [ 9: 0] tag_vpn_1_r      [0 : TLB_ENTRIES - 1];
reg  [ 9: 0] tag_vpn_0_r      [0 : TLB_ENTRIES - 1];
 
reg  [31: 0] content_r        [0 : TLB_ENTRIES - 1];
wire [11: 0] content_ppn_1    [0 : TLB_ENTRIES - 1];
wire [ 9: 0] content_ppn_0    [0 : TLB_ENTRIES - 1];
wire [ 1: 0] content_rsw      [0 : TLB_ENTRIES - 1];
wire         content_D        [0 : TLB_ENTRIES - 1];
wire         content_A        [0 : TLB_ENTRIES - 1];
wire         content_G        [0 : TLB_ENTRIES - 1];
wire         content_U        [0 : TLB_ENTRIES - 1];
wire         content_X        [0 : TLB_ENTRIES - 1];
wire         content_W        [0 : TLB_ENTRIES - 1];
wire         content_R        [0 : TLB_ENTRIES - 1];
wire         content_V        [0 : TLB_ENTRIES - 1];        
 
reg          content_is_4MB_r [0 : TLB_ENTRIES - 1];
reg          content_valid_r  [0 : TLB_ENTRIES - 1];



wire [ 9: 0] vaddr_i_vpn_1;
wire [ 9: 0] vaddr_i_vpn_0;


reg [0 : TLB_ENTRIES - 1] tlb_sel;
reg [33: 0]               tlb_paddr [0 : TLB_ENTRIES - 1];

reg [$clog2(TLB_ENTRIES) - 1 : 0] FIFO_cnt_r;
//=======================================================
// User Logic                         
//=======================================================
//-----------------------------------------------
// Translate virtual address to physical address
//-----------------------------------------------
// --------------------------
// | vpn_1 | vpn_0 | offset |
// --------------------------
// |31   22|21   12|11     0|
// --------------------------
// --------------------------
// | ppn_1 | ppn_0 | offset |
// --------------------------
// |33   22|21   12|11     0|
// --------------------------
//vpn
assign vaddr_i_vpn_1 = vaddr_i[31:22];
assign vaddr_i_vpn_0 = vaddr_i[21:12];

always@(*) begin
    for(i = 0; i < TLB_ENTRIES; i = i + 1) begin
        if(content_valid_r[i] && (tag_vpn_1_r[i] == vaddr_i_vpn_1)) begin
            if(content_is_4MB_r[i]) begin
                tlb_sel[i]   = 'b1;
                tlb_paddr[i] = {content_ppn_1[i], vaddr_i[21:0]};
            end else begin
                if(tag_vpn_0_r[i] == vaddr_i_vpn_0) begin
                    tlb_sel[i]   = 'b1;
                    tlb_paddr[i] = {content_ppn_1[i], content_ppn_0[i], vaddr_i[11:0]};
                end else begin
                    tlb_sel[i]   = 'b0;
                    tlb_paddr[i] = 'd0;
                end
            end    
        end else begin
            tlb_sel[i]   = 'b0;
            tlb_paddr[i] = 'd0;
        end
    end
end

//-----------------------------------------------
// Update the tlb
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------
// | ppn_1 | ppn_0 | rsw | dirty | accessed | global | user | executable | writable | readable | valid |
// -----------------------------------------------------------------------------------------------------
// |31   20|19   10|9   8|   7   |    6     |   5    |  4   |     3      |     2    |     1    |   0   |
// -----------------------------------------------------------------------------------------------------

//content valid
always@(posedge clk_i) begin
    if(rst_i) begin
        for(i = 0; i < TLB_ENTRIES; i = i +1) begin
            content_valid_r[i]    <= 'b0;
        end
    end else if(flush_i) begin
        for(i = 0; i < TLB_ENTRIES; i = i +1) begin
            content_valid_r[i]    <= 'b0;
        end
    end else if(update_vld_i) begin
        content_valid_r[FIFO_cnt_r] <= 'b1;
    end
end

//content
always@(posedge clk_i) begin
    if(update_vld_i) begin
        content_r[FIFO_cnt_r] <= update_content_i;
    end
end

generate
    for(g = 0; g < TLB_ENTRIES ; g = g + 1) 
    begin : content_gen
        assign content_ppn_1[g] = content_r[g][31:20];
        assign content_ppn_0[g] = content_r[g][19:10];
        assign content_rsw[g]   = content_r[g][ 9: 8];
        assign content_D[g]     = content_r[g][7];
        assign content_A[g]     = content_r[g][6];
        assign content_G[g]     = content_r[g][5];
        assign content_U[g]     = content_r[g][4];
        assign content_X[g]     = content_r[g][3];
        assign content_W[g]     = content_r[g][2];
        assign content_R[g]     = content_r[g][1];
        assign content_V[g]     = content_r[g][0];
    end
endgenerate

//content is 4MB
always@(posedge clk_i) begin
    if(update_vld_i) begin
        content_is_4MB_r[FIFO_cnt_r] <= update_content_is_4MB_i;
    end
end

//tag
always@(posedge clk_i) begin
    if(rst_i) begin
        for(i = 0; i < TLB_ENTRIES; i = i +1) begin
            //if not reset, translate may error, output will be unknown
            tag_vpn_1_r[i] <= 'd0;
            tag_vpn_0_r[i] <= 'd0;
        end
    end else if(update_vld_i) begin
        tag_vpn_1_r[FIFO_cnt_r] <= update_tag_i[19:10];
        tag_vpn_0_r[FIFO_cnt_r] <= update_tag_i[ 9: 0];
    end
end
//-----------------------------------------------
// FIFO replace policy signals
//-----------------------------------------------
always @(posedge clk_i) begin
    if (rst_i) begin
        FIFO_cnt_r <= 'd0;
    end else if (update_vld_i) begin
        FIFO_cnt_r <= FIFO_cnt_r + 'd1;
    end
end

//=======================================================
// Output signals interface                       
//=======================================================
//hit_o
assign hit_o = (|tlb_sel);

//paddr_o
// always@(*) begin
//     paddr_o = 'd0;
//     for(i = 0; i < TLB_ENTRIES; i = i + 1) begin
//         if(tlb_sel[i])
//             paddr_o = tlb_paddr[i];
//     end
// end
generate
    for(g = 0; g < TLB_ENTRIES ; g = g + 1) 
    begin : paddr_o_gen
        wire [33:0] paddr_temp;
        if(g == 0)
            assign paddr_temp = {(34){tlb_sel[g]}} & tlb_paddr[g];
        else
            assign paddr_temp = paddr_o_gen[g-1].paddr_temp | ( {(34){tlb_sel[g]}} & tlb_paddr[g] );
    end
    assign paddr_o = paddr_o_gen[TLB_ENTRIES-1].paddr_temp[31:0];
endgenerate

//content_o
generate
    for(g = 0; g < TLB_ENTRIES ; g = g + 1) 
    begin : content_o_gen
        wire [31:0] content_temp;
        if(g == 0)
            assign content_temp = {(32){tlb_sel[g]}} & content_r[g];
        else
            assign content_temp = content_o_gen[g-1].content_temp | ( {(32){tlb_sel[g]}} & content_r[g] );
    end
    assign content_o = content_o_gen[TLB_ENTRIES-1].content_temp;
endgenerate

endmodule