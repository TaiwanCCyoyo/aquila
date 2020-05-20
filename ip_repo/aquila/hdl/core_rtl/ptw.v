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

module ptw (
    input  wire         clk_i,
    input  wire         rst_i,

    // Flush
    input  wire         flush_i,

    // Update ITLB
    input  wire         itlb_miss_i,
    input  wire [31: 0] i_vaddr_i,

    output wire         itlb_update_vld_o,
    output wire [31: 0] itlb_update_content_o,
    output wire         itlb_update_content_is_4MB_o,
    
    // Update DTLB
    input  wire         dtlb_miss_i,
    input  wire [31: 0] d_vaddr_i,

    output wire         dtlb_update_vld_o,
    output wire [31: 0] dtlb_update_content_o,
    output wire         dtlb_update_content_is_4MB_o,

    // To D$
    output wire         req_vld_o,         // send a request.
    output wire         req_rw_o,          // 0 for read, 1 for write.
    output wire [3 : 0] req_byte_enable_o, // Byte-enable signal.
    output wire [31: 0] req_addr_o,        // Memory address for the request.

    // From D$
    input  wire         rtrn_vld_i,       // The cache data is ready.
    input  wire [31: 0] rtrn_data_i,      // Data from main memory.

    // From CSR
    input  wire [21: 0] root_ppn_i,
    input  wire [ 8: 0] asid_i,
    input  wire [ 1: 0] privilege_lvl_i,

    // Eception
    output wire         i_exp_vld_o,

    output wire         d_exp_vld_o
);

//=======================================================
// Parameter and Integer
//=======================================================
parameter [1:0] Idle    = 0,
                Look_up = 1,
                Update  = 2;
//=======================================================
// Wire and Reg 
//=======================================================

//Look up FSM
reg [ 1: 0] S, S_nxt;
reg         work_for_itlb_r;
reg [33: 0] req_addr_r;
reg         pte_lvl_r;

wire [ 9 : 0] d_vaddr_i_vpn_1, d_vaddr_i_vpn_0;
wire [ 9 : 0] i_vaddr_i_vpn_1, i_vaddr_i_vpn_0;


//-----------------------------------------------
// PTE page table entry
//-----------------------------------------------
wire [11: 0] rtrn_data_ppn_1;
wire [ 9: 0] rtrn_data_ppn_0;
wire [ 1: 0] rtrn_data_rsw;
wire         rtrn_data_D;
wire         rtrn_data_A;
wire         rtrn_data_G;
wire         rtrn_data_U;
wire         rtrn_data_X;
wire         rtrn_data_W;
wire         rtrn_data_R;
wire         rtrn_data_V;

reg  [31: 0] rtrn_data_r;
reg          content_is_4MB_r;

//Exception
reg         exception_vld;

//=======================================================
// User Logic                         
//=======================================================
// --------------------------
// | vpn_1 | vpn_0 | offset |
// --------------------------
// |31   22|21   12|11     0|
// --------------------------
//vpn
assign i_vaddr_i_vpn_1 = i_vaddr_i[31:22];
assign i_vaddr_i_vpn_0 = i_vaddr_i[21:12];
assign d_vaddr_i_vpn_1 = d_vaddr_i[31:22];
assign d_vaddr_i_vpn_0 = d_vaddr_i[21:12];

//-----------------------------------------------
// PTE page table entry
//-----------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
// | ppn_1 | ppn_0 | rsw | D(dirty) | A(accessed) | G(global) | U(user) | X(executable) | W(writable) | R(readable) | V(valid) |
// -----------------------------------------------------------------------------------------------------------------------------
// |31   20|19   10|9   8|    7     |      6      |     5     |    4    |       3       |      2      |      1      |     0    |
// -----------------------------------------------------------------------------------------------------------------------------
assign rtrn_data_ppn_1      = rtrn_data_i[31:20];
assign rtrn_data_ppn_0      = rtrn_data_i[19:10];
assign rtrn_data_rsw        = rtrn_data_i[ 9: 8];
assign rtrn_data_D          = rtrn_data_i[7];
assign rtrn_data_A          = rtrn_data_i[6];
assign rtrn_data_G          = rtrn_data_i[5];
assign rtrn_data_U          = rtrn_data_i[4];
assign rtrn_data_X          = rtrn_data_i[3];
assign rtrn_data_W          = rtrn_data_i[2];
assign rtrn_data_R          = rtrn_data_i[1];
assign rtrn_data_V          = rtrn_data_i[0];
//-----------------------------------------------
// MMU req
//-----------------------------------------------
assign req_vld_o         = (S == Look_up && !rtrn_vld_i);
assign req_rw_o          = 'b0; //read only
assign req_byte_enable_o = 'b1111;
assign req_addr_o        = req_addr_r[31:0];

//-----------------------------------------------
// PTW Finite State Machine
//-----------------------------------------------
always@(posedge clk_i) begin
    if(rst_i) begin
        S <= 'd0;
    end else begin
        S <= S_nxt;
    end
end

always @(*) begin
    case(S)
        Idle:
            if(itlb_miss_i || dtlb_miss_i) begin
                S_nxt = Look_up;
            end else begin
                S_nxt = Idle;
            end
        Look_up:
            if(rtrn_vld_i) begin
                if(!rtrn_data_V || 
                    (rtrn_data_W && !rtrn_data_R)) begin
                        //page fault
                        S_nxt = Idle;
                end else begin
                    if(rtrn_data_X || rtrn_data_R) begin
                        //leaf page
                        S_nxt = Update;
                    end else begin
                        if(!pte_lvl_r) begin
                            //Pointer to next level
                            S_nxt = Look_up;
                        end else begin
                            //Page fault
                            S_nxt = Idle;
                        end 
                    end
                end
            end else begin
                S_nxt = Look_up;
            end 
        Update:
            S_nxt = Idle;
    endcase
end

//-----------------------------------------------
// Look up PTE
//-----------------------------------------------

// work_for_itlb_r
always@(posedge clk_i) begin
    if(S == Idle) begin
        if(itlb_miss_i) begin
            work_for_itlb_r <= 1'b1; //itlb miss
        end else begin
            work_for_itlb_r <= 1'b0; //dtlb miss or nothing
        end
    end
end

// ----------------------------------------------------
// | X | W | R | Meaning                              |
// |---|---|---|--------------------------------------|
// | 0 | 0 | 0 | Pointer to next level of page table. |
// | 0 | 0 | 1 | Read-only page.                      |
// | 0 | 1 | 0 | Reserved for future use.             |
// | 0 | 1 | 1 | Read-write page.                     |
// | 1 | 0 | 0 | Execute-only page.                   |
// | 1 | 0 | 1 | Read-execute page.                   |
// | 1 | 1 | 0 | Reserved for future use.             |
// | 1 | 1 | 1 | Read-write-execute page.             |
// ----------------------------------------------------

//req_addr_r
always@(posedge clk_i) begin
    if(S == Idle) begin
        if(itlb_miss_i) begin
            req_addr_r <= {root_ppn_i, i_vaddr_i_vpn_1, 2'b0}; //itlb miss
        end else begin
            req_addr_r <= {root_ppn_i, d_vaddr_i_vpn_1, 2'b0}; //dtlb miss or nothing
        end
    end else if(S == Look_up) begin
        if(rtrn_vld_i) begin
            if(rtrn_data_V
            && !(rtrn_data_W && !rtrn_data_R) //not reserved page
            && !(rtrn_data_X || rtrn_data_R) //not leaf page
            && (!pte_lvl_r)) begin //level 1               
                if(work_for_itlb_r)
                    req_addr_r <= {rtrn_data_i[31:10], i_vaddr_i_vpn_0, 2'b0};
                else
                    req_addr_r <= {rtrn_data_i[31:10], d_vaddr_i_vpn_0, 2'b0};
            end
        end
    end
end

// content_is_4MB
always@(posedge clk_i) begin
    if(rtrn_vld_i) begin
        if(rtrn_data_V 
        && !(rtrn_data_W && !rtrn_data_R)//not reserved page
        && (rtrn_data_X || rtrn_data_R)) begin //leaf page
            //leaf page
            if(!pte_lvl_r) begin
                content_is_4MB_r <= 1'b1;
            end else begin
                content_is_4MB_r <= 1'b0;
            end
        end
    end
end

//return data
always@(posedge clk_i) begin
    rtrn_data_r <= rtrn_data_i;
end

//pte_lvl_r
always@(posedge clk_i) begin
    if(S == Idle) begin
        pte_lvl_r <= 'b0;
    end else if(S == Look_up && rtrn_vld_i) begin
        pte_lvl_r <= 'b1;
    end
end

//=======================================================
// Output signals interface                       
//=======================================================
//-----------------------------------------------
// Update TLB
//-----------------------------------------------
assign itlb_update_vld_o     = (S == Update) && work_for_itlb_r;
assign itlb_update_content_o        = rtrn_data_r;
assign itlb_update_content_is_4MB_o = content_is_4MB_r;


assign dtlb_update_vld_o     = (S == Update) && ~work_for_itlb_r;
assign dtlb_update_content_o        = rtrn_data_r;
assign dtlb_update_content_is_4MB_o = content_is_4MB_r;
//-----------------------------------------------
// Exception
//-----------------------------------------------
assign i_exp_vld_o   = exception_vld & work_for_itlb_r;

assign d_exp_vld_o   = exception_vld & (~work_for_itlb_r);



always@(*) begin
    exception_vld = 0;
    if(S == Look_up) begin
        if(rtrn_vld_i) begin
            if(~rtrn_data_V || 
                (rtrn_data_W && ~rtrn_data_R)) begin
                    //page fault
                    exception_vld = 1;
            end else begin
                if(rtrn_data_X || rtrn_data_R) begin
                    //leaf page
                    exception_vld = 0;
                end else begin
                    if(!pte_lvl_r) begin
                        //Pointer to next level
                        exception_vld = 0;
                    end else begin
                        //Page fault
                        exception_vld = 1;
                    end 
                end
            end
        end
    end
end

endmodule