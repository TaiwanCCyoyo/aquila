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
    input  wire         flush_i,

    //From CSR
    input  wire         enable_i,
    input  wire [21: 0] root_ppn_i,
    input  wire [ 8: 0] asid_i,
    input  wire [ 1: 0] privilege_lvl_i,

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

    //From dcache
    input  wire         d_rtrn_vld_i,
    input  wire [31: 0] d_rtrn_data_i,

    //From dcache to processor
    output wire         d_rtrn_vld_o,
    output wire [31: 0] d_rtrn_data_o,

    //Exception
    output wire         i_exception_vld_o,
    output wire [ 3: 0] i_exception_cause_o,

    output wire         d_exception_vld_o,
    output wire [ 3: 0] d_exception_cause_o
);
    //=======================================================
    // Parameter and Integer
    //=======================================================

    //=======================================================
    // Wire and Reg 
    //=======================================================
    // --------------
    // itlb
    // --------------
    wire         hit_itlb;
    wire [31: 0] paddr_itlb;


    // --------------
    // dtlb
    // --------------
    wire         hit_dtlb;
    wire [31: 0] paddr_dtlb;       // Memory address.
    

    // --------------
    // itlb <-> ptw
    // --------------
    wire         update_vld_ptw2itlb;
    wire [31: 0] update_content_ptw2itlb;
    wire         update_content_is_4MB_ptw2itlb;

    // --------------
    // dtlb <-> ptw
    // --------------
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
    // -------------------
    // PMA


    // -------------------
    // PMP


    // ------------------------------------------------------
    // Data
    // ------------------------------------------------------
    // -------------------
    // PMA

    // -------------------
    // PMP

    // -------------------
    // To Arbitrate
    assign req_vld_d2arb           = hit_dtlb;
    assign req_paddr_d2arb         = paddr_dtlb;
    assign req_data_d2arb          = d_req_data_i;
    assign req_rw_d2arb            = d_req_rw_i;
    assign req_byte_enable_d2arb   = d_req_byte_enable_i;
    

    //=======================================================
    // Output signals interface                       
    //=======================================================
    assign i_req_vld_o   = hit_itlb;
    assign i_req_paddr_o = paddr_itlb;

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
        .update_tag_i(itlb_miss_addr_r),
        .update_content_i(update_content_ptw2itlb),
        .update_content_is_4MB_i(update_content_is_4MB_ptw2itlb),
        
        // Translate addres
        .translate_req_vld_i(i_req_vld_i),
        .asid_i(),
        .vaddr_i(i_req_vaddr_i),
        .hit_o(hit_itlb),
        .paddr_o(paddr_itlb)
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
        .update_tag_i(dtlb_miss_addr_r),
        .update_content_i(update_content_ptw2dtlb),
        .update_content_is_4MB_i(update_content_is_4MB_ptw2dtlb),
        
        // Translate addres
        .translate_req_vld_i(d_req_vld_i),
        .asid_i(),
        .vaddr_i(d_req_vaddr_i),
        .hit_o(hit_dtlb),
        .paddr_o(paddr_dtlb)
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
        .itlb_miss_i(itlb_miss_vld_r),
        .i_vaddr_i(itlb_miss_addr_r),

        .itlb_update_vld_o(update_vld_ptw2itlb),
        .itlb_update_content_o(update_content_ptw2itlb),
        .itlb_update_content_is_4MB_o(update_content_is_4MB_ptw2itlb),
        
        // Update DTLB
        .dtlb_miss_i(dtlb_miss_vld_r),
        .d_vaddr_i(dtlb_miss_addr_r),

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
        .exception_vld_o(),
        .exception_cause_o()
    );

    // ------------------------------------------------------
    // Arbitrate
    // ------------------------------------------------------
    // reg          req_vld_temp_r;
    // wire         req_vld_temp = req_vld_temp_r && ~rtrn_vld_temp;
    // wire [31: 0] req_addr_temp = 'd0;
    // wire         req_rw_temp = 'd0;
    // wire [ 3: 0] req_byte_enable_temp = 'd15;

    // wire         rtrn_vld_temp;
    // wire [31: 0] rtrn_data_temp;

    // reg [31: 0] cnt_temp;

    // always@(posedge clk_i) begin
    //     if(rst_i) begin
    //         req_vld_temp_r <= 'b0;
    //     end else begin
    //         if(req_vld_temp_r == 'b1) begin
    //             if(rtrn_vld_temp)
    //                 req_vld_temp_r <= 'd0;
    //             else
    //                 req_vld_temp_r <= 'd1;
    //         end else begin
    //             if(cnt_temp == 'd1000)
    //                 req_vld_temp_r <= 'd1;
    //         end
    //     end
    // end

    // always@(posedge clk_i) begin
    //     if(rst_i) begin
    //         cnt_temp <= 'd0;
    //     end else begin
    //         if(cnt_temp == 'd1000)
    //             cnt_temp <= 'd0;
    //         else
    //             cnt_temp <= cnt_temp + 1;
    //     end
    // end


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

        // .ptw_req_vld_i(req_vld_temp),    
        // .ptw_req_paddr_i(req_addr_temp),    
        // .ptw_req_data_i('d0),
        // .ptw_req_rw_i(req_rw_temp),      
        // .ptw_req_byte_enable_i(req_byte_enable_temp),

        // .ptw_rtrn_vld_o(rtrn_vld_temp),
        // .ptw_rtrn_data_o(rtrn_data_temp),

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