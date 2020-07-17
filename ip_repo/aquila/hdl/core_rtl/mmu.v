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

module mmu #(
    parameter INSTR_TLB_ENTRIES     = 4,
    parameter DATA_TLB_ENTRIES      = 4,
    parameter ASID_WIDTH            = 1
) (
    input  wire         clk_i,
    input  wire         rst_i,

    //
    input  wire         flush_i,
    input  wire         flush_type_i,
    input  wire [31: 0] fulsh_vaddr_i,
    input  wire [31: 0] fulsh_asid_i,

    //From CSR
    input  wire         enable_i,
    input  wire [21: 0] root_ppn_i,
    input  wire [ 8: 0] asid_i,
    input  wire [ 1: 0] privilege_lvl_i,
    input  wire [ 1: 0] ld_st_privilege_lvl_i,
    input  wire         mxr_i,
    input  wire         sum_i,

    //From processor to icache
    input  wire         i_req_vld_i,         // request.
    input  wire [31: 0] i_req_vaddr_i,       // Memory address.

    output wire         i_req_vld_o,         // request.
    output wire [31: 0] i_req_paddr_o,        // Memory address.

    //From processor to dcache
    input  wire         d_req_vld_i,         // request.
    input  wire [31: 0] d_req_vaddr_i,       // Memory address.
    input  wire [31: 0] d_req_data_i,
    input  wire         d_req_rw_i,          // 0: data read, 1: data write.
    input  wire [ 3: 0] d_req_byte_enable_i,
    
    //Send request to dcache
    output wire         d_req_vld_o,
    // output wire         d_strobe_o,
    output wire [31: 0] d_req_paddr_o,
    output wire [31: 0] d_req_data_o,
    output wire         d_req_rw_o,          // 0: data read, 1: data write.
    output wire [ 3: 0] d_req_byte_enable_o,

    //From icache
    input  wire         i_rtrn_vld_i,
    input  wire [31: 0] i_rtrn_data_i,

    //From icache to processor
    output wire         i_rtrn_vld_o,
    output wire [31: 0] i_rtrn_data_o,
    output wire [31: 0] i_rtrn_vaddr_o,
    // output wire [31: 0] i_rtrn_paddr_o,

    //From dcache
    input  wire         d_rtrn_vld_i,
    input  wire [31: 0] d_rtrn_data_i,

    //From dcache to processor
    output wire         d_rtrn_vld_o,
    output wire [31: 0] d_rtrn_data_o,

    //Exception
    output wire         i_exp_vld_o,
    output wire [ 3: 0] i_exp_cause_o,
    output wire [31: 0] i_exp_tval_o,

    output wire         d_exp_vld_o,
    output wire [ 3: 0] d_exp_cause_o,
    output wire [31: 0] d_exp_tval_o
);
    //=======================================================
    // Parameter and Integer
    //=======================================================

    //=======================================================
    // Wire and Reg 
    //=======================================================
    // -----------------------------------------------------------------------------------------------------------------------------
    // | ppn_1 | ppn_0 | rsw | D(dirty) | A(accessed) | G(global) | U(user) | X(executable) | W(writable) | R(readable) | V(valid) |
    // -----------------------------------------------------------------------------------------------------------------------------
    // |31   20|19   10|9   8|    7     |      6      |     5     |    4    |       3       |      2      |      1      |     0    |
    // -----------------------------------------------------------------------------------------------------------------------------
    // --------------
    // itlb
    // --------------
    reg          itlb_req_vld_r;
    reg  [31: 0] itlb_paddr_r;
    reg  [31: 0] itlb_vaddr_r;
    reg          itlb_content_U_r;
    reg          itlb_content_R_r;
    reg          itlb_content_X_r;

    reg          pmp_i_exp_vld;
    reg  [ 3: 0] i_exp_cause;
    reg  [31: 0] i_exp_tval;
    wire         hit_itlb;
    wire [31: 0] paddr_itlb;
    wire [31: 0] itlb_content;


    // --------------
    // dtlb
    // --------------
    reg          dtlb_req_vld_r;
    reg  [31: 0] dtlb_paddr_r;
    reg          dtlb_content_U_r;
    reg          dtlb_content_R_r;
    reg          dtlb_content_W_r;
    reg          dtlb_content_D_r;
    reg          dtlb_content_X_r;

    reg          pmp_d_exp_vld;
    reg  [ 3: 0] d_exp_cause;
    reg  [31: 0] d_exp_tval;
    wire         hit_dtlb;
    wire [31: 0] paddr_dtlb;       // Memory address.
    wire [31: 0] dtlb_content;
    

    // --------------
    // itlb <-> ptw
    // --------------
    wire         itlb_miss_vld = (enable_i && !hit_itlb && i_req_vld_i);

    wire         update_vld_ptw2itlb;
    wire [31: 0] update_content_ptw2itlb;
    wire         update_content_is_4MB_ptw2itlb;

    // --------------
    // dtlb <-> ptw
    // --------------
    wire         dtlb_miss_vld = (enable_i && !hit_dtlb && d_req_vld_i);

    wire         update_vld_ptw2dtlb;
    wire [31: 0] update_content_ptw2dtlb;
    wire         update_content_is_4MB_ptw2dtlb;

    // ----------------
    // dtlb <-> mmu_arb
    // ----------------
    wire         req_vld_d2arb;   
    wire [31: 0] req_paddr_d2arb;
    wire [31: 0] req_data_d2arb;
    wire         req_rw_d2arb;
    wire [ 3: 0] req_byte_enable_d2arb;

    // --------------
    // ptw <-> mmu_arb
    // --------------
    wire         req_vld_ptw2arb;
    wire         req_rw_ptw2arb;
    wire [ 3: 0] req_byte_enable_ptw2arb;
    wire [31: 0] req_addr_ptw2arb;

    wire         rtrn_vld_arb2ptw;
    wire [31: 0] rtrn_data_arb2ptw;

    //=======================================================
    // User Logic                         
    //=======================================================
    // ------------------------------------------------------
    // Instruction
    // ------------------------------------------------------
    always@(posedge clk_i) begin
        if(rst_i) begin
            itlb_req_vld_r   <= 0;
            itlb_paddr_r     <= 0;
            itlb_vaddr_r     <= 0;
            itlb_content_U_r <= 0;
            itlb_content_R_r <= 0;
            itlb_content_X_r <= 0;
        end if(enable_i && hit_itlb && i_req_vld_i && !flush_i) begin
            itlb_req_vld_r   <= 1;
            itlb_paddr_r     <= paddr_itlb;
            itlb_vaddr_r     <= i_req_vaddr_i;
            itlb_content_U_r <= itlb_content[4];
            itlb_content_R_r <= itlb_content[1];
            itlb_content_X_r <= itlb_content[3];
        end else if(i_rtrn_vld_i || i_exp_vld_o) begin
            itlb_req_vld_r   <= 0;
            itlb_paddr_r     <= 0;
            itlb_vaddr_r     <= 0;
            itlb_content_U_r <= 0;
            itlb_content_R_r <= 0;
            itlb_content_X_r <= 0;
        end
    end
    // -------------------
    // PMP
    always@(*) begin
        if(itlb_req_vld_r && 
            ((!itlb_content_X_r && !itlb_content_R_r)  || 
             (privilege_lvl_i == 'd0 && !itlb_content_U_r) || 
             (privilege_lvl_i == 'd1 &&  itlb_content_U_r)) ) begin
            pmp_i_exp_vld   = 1'b1;
        end else begin
            pmp_i_exp_vld   = 1'b0;
        end
    end

    always@(*) begin
        i_exp_cause = 'd12;
        i_exp_tval  = i_req_vaddr_i;
    end
    

    // -------------------
    // PMA


    // ------------------------------------------------------
    // Data
    // ------------------------------------------------------
    reg update_dtlb_done;
    always@(posedge clk_i) begin
        if(rst_i) begin
            update_dtlb_done <= 0;
        end else begin
            update_dtlb_done <= update_vld_ptw2dtlb;
        end
    end

    always@(posedge clk_i) begin
        if(rst_i) begin
            dtlb_req_vld_r   <= 0;
            dtlb_paddr_r     <= 0;
            dtlb_content_U_r <= 0;
            dtlb_content_R_r <= 0;
            dtlb_content_W_r <= 0;
            dtlb_content_D_r <= 0;
            dtlb_content_X_r <= 0;
        end if(enable_i && hit_dtlb && (d_req_vld_i || update_dtlb_done)) begin
            dtlb_req_vld_r   <= 1;
            dtlb_paddr_r     <= paddr_dtlb;
            dtlb_content_U_r <= dtlb_content[4];
            dtlb_content_R_r <= dtlb_content[1];
            dtlb_content_W_r <= dtlb_content[2];
            dtlb_content_D_r <= dtlb_content[7];
            dtlb_content_X_r <= dtlb_content[3];
        end else if(d_rtrn_vld_o || d_exp_vld_o) begin
            dtlb_req_vld_r   <= 0;
            dtlb_paddr_r     <= 0;
            dtlb_content_U_r <= 0;
            dtlb_content_R_r <= 0;
            dtlb_content_W_r <= 0;
            dtlb_content_D_r <= 0;
            dtlb_content_X_r <= 0;
        end
    end

    // -------------------
    // PMP
    always@(*) begin
        if(dtlb_req_vld_r && 
            ((!mxr_i && !dtlb_content_R_r) ||
             (mxr_i  && !dtlb_content_R_r && !dtlb_content_X_r) ||
             (d_req_rw_i && (!dtlb_content_W_r || !dtlb_content_D_r)) ||
             (              ld_st_privilege_lvl_i == 0   && !dtlb_content_U_r) || 
             (sum_i == 0 && ld_st_privilege_lvl_i == 'd1 && dtlb_content_U_r)) ) begin
                pmp_d_exp_vld   = 1'b1;
        end else begin
            pmp_d_exp_vld   = 1'b0;
        end
    end
        
    always@(*) begin
        d_exp_tval = d_req_vaddr_i;
        if(d_req_rw_i) begin
            d_exp_cause = 'd15;
        end else begin
            d_exp_cause = 'd13;
        end
    end

    // -------------------
    // PMA

    // -------------------
    // To Arbitrate
    assign req_vld_d2arb           = (enable_i)?(!d_exp_vld_o && dtlb_req_vld_r):d_req_vld_i;
    assign req_paddr_d2arb         = (enable_i)?dtlb_paddr_r                    :d_req_vaddr_i;
    assign req_data_d2arb          = d_req_data_i;
    assign req_rw_d2arb            = d_req_rw_i;
    assign req_byte_enable_d2arb   = d_req_byte_enable_i;
    

    //=======================================================
    // Output signals interface                       
    //=======================================================
    assign i_req_vld_o   = (enable_i)?(!i_exp_vld_o && !i_rtrn_vld_o && itlb_req_vld_r):i_req_vld_i;
    assign i_req_paddr_o = (enable_i)?itlb_paddr_r                    :i_req_vaddr_i;

    assign i_rtrn_vld_o   = i_rtrn_vld_i;
    assign i_rtrn_data_o  = i_rtrn_data_i;
    assign i_rtrn_vaddr_o = (enable_i && !ptw_i_exp_vld)?itlb_vaddr_r                    :i_req_vaddr_i;
    // assign i_rtrn_paddr_o = (enable_i && !ptw_i_exp_vld)?itlb_paddr_r                    :i_req_vaddr_i;


    assign i_exp_vld_o   = pmp_i_exp_vld || ptw_i_exp_vld;
    assign i_exp_cause_o = i_exp_cause;
    assign i_exp_tval_o  = i_exp_tval; 

    assign d_exp_vld_o   = pmp_d_exp_vld || ptw_d_exp_vld;
    assign d_exp_cause_o = d_exp_cause;
    assign d_exp_tval_o  = d_exp_tval; 
 
    //=======================================================
    // Other module                      
    //=======================================================

    //-----------------------------------------------
    // Translation Lookaside Buffer
    //-----------------------------------------------
    tlb #(
    ) itlb (
        .clk_i(clk_i),
        .rst_i(rst_i),
        
        // Flush
        .flush_i(flush_i),

        //Enable
        .enable_i(enable_i),
        
        // Update TLB
        .update_vld_i(update_vld_ptw2itlb),
        .update_tag_i(i_req_vaddr_i[31:12]),
        .update_content_i(update_content_ptw2itlb),
        .update_content_is_4MB_i(update_content_is_4MB_ptw2itlb),
        
        // Translate addres
        .asid_i(),
        .vaddr_i(i_req_vaddr_i),
        .hit_o(hit_itlb),
        .paddr_o(paddr_itlb),

        .content_o(itlb_content)
    );

    tlb #(
    ) dtlb (
        .clk_i(clk_i),
        .rst_i(rst_i),
        
        // Flush
        .flush_i(flush_i),
        
        //Enable
        .enable_i(enable_i),
        
        // Update TLB
        .update_vld_i(update_vld_ptw2dtlb),
        .update_tag_i(d_req_vaddr_i[31:12]),
        .update_content_i(update_content_ptw2dtlb),
        .update_content_is_4MB_i(update_content_is_4MB_ptw2dtlb),
        
        // Translate addres
        .asid_i(),
        .vaddr_i(d_req_vaddr_i),
        .hit_o(hit_dtlb),
        .paddr_o(paddr_dtlb),

        .content_o(dtlb_content)
    );
    
    //-----------------------------------------------
    // Page Table Walk
    //-----------------------------------------------
    ptw #(

    ) Ptw (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // Flush
        .flush_i(flush_i),

        // Update ITLB
        .itlb_miss_i(itlb_miss_vld),
        .i_vaddr_i(i_req_vaddr_i),

        .itlb_update_vld_o(update_vld_ptw2itlb),
        .itlb_update_content_o(update_content_ptw2itlb),
        .itlb_update_content_is_4MB_o(update_content_is_4MB_ptw2itlb),
        
        // Update DTLB
        .dtlb_miss_i(dtlb_miss_vld),
        .d_vaddr_i(d_req_vaddr_i),

        .dtlb_update_vld_o(update_vld_ptw2dtlb),
        .dtlb_update_content_o(update_content_ptw2dtlb),
        .dtlb_update_content_is_4MB_o(update_content_is_4MB_ptw2dtlb),

        // To D$
        .req_vld_o(req_vld_ptw2arb),        
        .req_addr_o(req_addr_ptw2arb),      
        .req_rw_o(req_rw_ptw2arb),          
        .req_byte_enable_o(req_byte_enable_ptw2arb),

        // From D$
        .rtrn_vld_i(rtrn_vld_arb2ptw),
        .rtrn_data_i(rtrn_data_arb2ptw),

        // From CSR
        .root_ppn_i(root_ppn_i),
        .asid_i(asid_i),
        .privilege_lvl_i(privilege_lvl_i),

        // Eception
        .i_exp_vld_o(ptw_i_exp_vld),
        .d_exp_vld_o(ptw_d_exp_vld)
    );

    // ------------------------------------------------------
    // Arbitrate
    // ------------------------------------------------------
    mmu_arb #(
    ) MMU_arb (
        .clk_i(clk_i),
        .rst_i(rst_i),

        // .enable_i(enable_i),

        // Flush
        .flush_i(flush_i),

        //From processor to dcache
        .d_req_vld_i(req_vld_d2arb),   
        .d_req_paddr_i(req_paddr_d2arb),    
        .d_req_data_i(req_data_d2arb),
        .d_req_rw_i(req_rw_d2arb),       
        .d_req_byte_enable_i(req_byte_enable_d2arb),

        .d_rtrn_vld_o(d_rtrn_vld_o),
        .d_rtrn_data_o(d_rtrn_data_o),

        //From ptw to dcache
        .ptw_req_vld_i(req_vld_ptw2arb),    
        .ptw_req_paddr_i(req_addr_ptw2arb),    
        .ptw_req_data_i('d0),
        .ptw_req_rw_i(req_rw_ptw2arb),      
        .ptw_req_byte_enable_i(req_byte_enable_ptw2arb),

        .ptw_rtrn_vld_o(rtrn_vld_arb2ptw),
        .ptw_rtrn_data_o(rtrn_data_arb2ptw),

        //To dcache
        .d_req_vld_o(d_req_vld_o),    
        // .d_strobe_o(d_strobe_o),
        .d_req_paddr_o(d_req_paddr_o),       
        .d_req_data_o(d_req_data_o),
        .d_req_rw_o(d_req_rw_o),      
        .d_req_byte_enable_o(d_req_byte_enable_o),

        .d_rtrn_vld_i(d_rtrn_vld_i),
        .d_rtrn_data_i(d_rtrn_data_i)
    );

endmodule