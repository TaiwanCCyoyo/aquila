`timescale 1 ns / 1 ps
// =============================================================================
//  Program : aquila_top.v
//  Author  : Chun-Jen Tsai
//  Date    : Oct/08/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the top-level Aquila IP wrapper for an AXI-based processor SoC.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  This module is based on the soc_top.v module written by Jin-you Wu
//  on Feb/28/2019. The original module was a stand-alone top-level module
//  for an SoC. This rework makes it a module embedded inside an AXI IP.
//
//  Jan/12/2020, by Chun-Jen Tsai:
//    Added a on-chip Tightly-Coupled Memory (TCM) to the aquila SoC.
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

module aquila_top #
(
    parameter integer ADDR_WIDTH      = 32, // Width of address bus
    parameter integer DATA_WIDTH      = 32, // Width of data bus
    parameter integer CACHE_LINE_SIZE = 256 // Size of a cache line in bits.
)
(
    input  clk_i,
    input  rst_i,   // level-sensitive reset signal.

    // Initial program counter address for the Aquila core
    input  [ADDR_WIDTH-1 : 0]      base_addr_i,

    // Aquila M_ICACHE master port interface signals
    output                         M_ICACHE_strobe_o,
    output [ADDR_WIDTH-1 : 0]      M_ICACHE_addr_o,
    input                          M_ICACHE_done_i,
    input  [CACHE_LINE_SIZE-1 : 0] M_ICACHE_data_i,

    // Aquila M_DCACHE master port interface signals
    output                         M_DCACHE_strobe_o,
    output [ADDR_WIDTH-1 : 0]      M_DCACHE_addr_o,
    output                         M_DCACHE_rw_o,
    output [CACHE_LINE_SIZE-1 : 0] M_DCACHE_data_o,
    input                          M_DCACHE_done_i,
    input  [CACHE_LINE_SIZE-1 : 0] M_DCACHE_data_i,

    // Aquila M_DEVICE master port interface signals
    output                         M_DEVICE_strobe_o,
    output [ADDR_WIDTH-1 : 0]      M_DEVICE_addr_o,
    output                         M_DEVICE_rw_o,
    output [DATA_WIDTH/8-1 : 0]    M_DEVICE_byte_enable_o,
    output [DATA_WIDTH-1 : 0]      M_DEVICE_data_o,
    input                          M_DEVICE_data_ready_i,
    input  [DATA_WIDTH-1 : 0]      M_DEVICE_data_i
);

// ------------- Signals for cpu, cache and master ip -------------------------
// CPU core
wire                         ins_sel;
wire [1 : 0]                 mem_sel;
wire                         data_rw;

// Processor to instruction memory signals.
wire                         p_i_req, p_i_ready;
wire [ADDR_WIDTH-1 : 0]      p_i_addr;
wire [DATA_WIDTH-1 : 0]      p_i_instr;

wire [DATA_WIDTH-1 : 0]      instr_from_tcm;
wire [DATA_WIDTH-1 : 0]      instr_from_cache;
wire                         tcm_i_ready;
wire                         cache_i_ready;

// Processor to data memory signals.
wire                         p_d_req, p_d_strobe, p_d_ready;
wire [ADDR_WIDTH-1 : 0]      p_d_addr;
wire [DATA_WIDTH/8-1 : 0]    p_d_byte_enable;
wire [DATA_WIDTH-1 : 0]      p_d_dout, p_d_din;

wire [DATA_WIDTH-1 : 0]      data_from_tcm;
wire [DATA_WIDTH-1 : 0]      data_from_cache;
wire                         tcm_d_ready;
wire                         cache_d_ready;

// I/D Caches to DDRx memory signals.
wire                         m_i_strobe, m_i_ready;
wire                         m_d_strobe, m_d_rw, m_d_ready;
wire [ADDR_WIDTH-1 : 0]      m_i_addr, m_d_addr;
wire [CACHE_LINE_SIZE-1 : 0] m_i_dout, m_d_din, m_d_dout;

// Interrupt signals.
wire tmr_irq, sft_irq;

// Core Local Interrupt controller (CLINT) memory read bus.
wire [DATA_WIDTH-1 : 0]      data_from_clint;
wire                         clint_d_ready;

// ----------- System Memory Map: DDRx DRAM, Devices, or CLINT --------------
//       [0] 0x0000_0000 - 0x0FFF_FFFF : Tightly-Coupled Memory (TCM)
//       [1] 0x8000_0000 - 0xBFFF_FFFF : DDRx DRAM memory (cached)
//       [2] 0xC000_0000 - 0xCFFF_FFFF : device memory (uncached)
//       [3] 0xF000_0000 - 0xF000_0010 : CLINT I/O registers (uncached)
//
wire [3 : 0] inst_seg, data_seg;

assign inst_seg = p_i_addr[ADDR_WIDTH-1:ADDR_WIDTH-4];
assign data_seg = p_d_addr[ADDR_WIDTH-1:ADDR_WIDTH-4];

assign ins_sel = (inst_seg == 4'h0)? 0 : 1;
assign mem_sel = (data_seg == 4'h0)? 0 :
                 (data_seg == 4'hC)? 2 :
                 (data_seg == 4'hF)? 3 : 1;

assign p_i_instr = (ins_sel == 0)? instr_from_tcm : instr_from_cache;
assign p_i_ready = (ins_sel == 0)? tcm_i_ready : cache_i_ready;

reg  [1:0] mem_sel_r;
always @(posedge clk_i) begin
	mem_sel_r <= mem_sel;
end

assign p_d_dout  = (mem_sel_r == 0)? data_from_tcm :
                   (mem_sel_r == 1)? data_from_cache :
                   (mem_sel_r == 2)? M_DEVICE_data_i : data_from_clint;
assign p_d_ready = (mem_sel_r == 0)? tcm_d_ready :
                   (mem_sel_r == 1)? cache_d_ready :
                   (mem_sel_r == 2)? M_DEVICE_data_ready_i : clint_d_ready;

// --- Master IP interface driving signals for I/D caches and I/O devices ---
assign M_ICACHE_strobe_o = m_i_strobe;
assign M_ICACHE_addr_o   = m_i_addr;
assign m_i_ready         = M_ICACHE_done_i;
assign m_i_dout          = M_ICACHE_data_i;

assign M_DCACHE_strobe_o = m_d_strobe;
assign M_DCACHE_addr_o   = m_d_addr;
assign M_DCACHE_rw_o     = m_d_rw;
assign M_DCACHE_data_o   = m_d_din;
assign m_d_ready         = M_DCACHE_done_i;
assign m_d_dout          = M_DCACHE_data_i;

assign M_DEVICE_strobe_o      = p_d_strobe && (mem_sel == 2);
assign M_DEVICE_addr_o        = (mem_sel == 2)? p_d_addr : 32'h0;
assign M_DEVICE_rw_o          = data_rw && (mem_sel == 2);
assign M_DEVICE_byte_enable_o = p_d_byte_enable;
assign M_DEVICE_data_o        = (mem_sel == 2)? p_d_din : 32'h0;

// ----------------------------------------------------------------------------
//  Aquila processor core
//
localparam COND_ENTRY_NUM    = 24,
           COND_DATA_WIDTH   = 32,
           UNCOND_ENTRY_NUM  = 20,
           UNCOND_DATA_WIDTH = 32;

core_top #(
    .HART_ID(0),
    .COND_ENTRY_NUM(COND_ENTRY_NUM),
    .COND_DATA_WIDTH(COND_DATA_WIDTH),
    .UNCOND_ENTRY_NUM(UNCOND_ENTRY_NUM),
    .UNCOND_DATA_WIDTH(UNCOND_DATA_WIDTH)
)
RISCV_CORE0(
    // System signals
    .clk_i(clk_i),
    .rst_i(rst_i),            // from slave register
    .stall_i(1'b0),         // disable user stall signal

    // Program counter address at reset for the Aquila core
    .init_pc_addr_i(base_addr_i),

    // Instruction port
    .instruction_i(p_i_instr),
    .instruction_ready_i(p_i_ready),
    .instruction_addr_o(p_i_addr),
    .instruction_req_o(p_i_req),

    // Data port
    .data_read_i(p_d_dout),
    .data_ready_i(p_d_ready),
    .data_write_o(p_d_din),
    .data_addr_o(p_d_addr),
    .data_rw_o(data_rw),
    .data_byte_enable_o(p_d_byte_enable),
    .data_req_o(p_d_req),
    .data_strobe_o(p_d_strobe),

    // Interrupts
    .ext_irq_i(1'b0),     // no external interrupt (yet)
    .tmr_irq_i(tmr_irq),
    .sft_irq_i(sft_irq)
);

// ----------------------------------------------------------------------------
//  Instiantiation of the dual-port tightly-coupled scratchpad memory module.
//  0x00000000 ~ 0x0FFFFFFF
localparam TCM_SIZE_IN_WORDS = 16384; // 64KB
localparam TCM_ADDR_WIDTH = $clog2(TCM_SIZE_IN_WORDS);

sram_dp #(.DATA_WIDTH(DATA_WIDTH), .N_ENTRIES(TCM_SIZE_IN_WORDS))
TCM(
    // Instruction
    .clk1_i(clk_i),
    .en1_i(p_i_req && (ins_sel == 0)),
    .we1_i(1'b0),
    .be1_i(4'b1111),
    .addr1_i(p_i_addr[TCM_ADDR_WIDTH+1 : 2]),
    .data1_i({DATA_WIDTH{1'b0}}),
    .data1_o(instr_from_tcm),
    .ready1_o(tcm_i_ready),

    // Data
    .clk2_i(clk_i),
    .en2_i(p_d_req && (mem_sel == 0)),
    .we2_i(data_rw && (mem_sel == 0)),
    .be2_i(p_d_byte_enable),
    .addr2_i(p_d_addr[TCM_ADDR_WIDTH+1 : 2]),
    .data2_i(p_d_din),  // data from processor write bus
    .data2_o(data_from_tcm),
    .ready2_o(tcm_d_ready)
);

// ----------------------------------------------------------------------------
//  Instiantiation of the CLINT module.
//
clint #( .TIMER(1_00_000) )
CLINT(
    .clk_i(clk_i),
    .rst_i(rst_i),
    .en_i(mem_sel == 3),
    .we_i(data_rw && (mem_sel == 3)),
    .addr_i({6'b0, p_d_addr[ADDR_WIDTH - 5 : 2]}),
    .data_i(p_d_din),
    .data_o(data_from_clint),
    .data_ready_o(clint_d_ready),

    .tmr_irq_o(tmr_irq),
    .sft_irq_o(sft_irq)
);

// ----------------------------------------------------------------------------
//  Instiantiation of the I/D-cache modules.
//
localparam ICACHE_SIZE = 4; // Cache size in KB.
localparam DCACHE_SIZE = 4; // Cache size in KB.

// Instruction read from I-cache port.
icache #(.ADDR_WIDTH(ADDR_WIDTH), .CACHE_SIZE(ICACHE_SIZE))
I_Cache(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .p_addr_i(p_i_addr),
    .p_req_i(p_i_req && (ins_sel == 1)),
    .p_instr_o(instr_from_cache),
    .p_ready_o(cache_i_ready),

    .m_addr_o(m_i_addr),
    .m_data_i(m_i_dout),
    .m_strobe_o(m_i_strobe),
    .m_ready_i(m_i_ready)
);

// Data read/write through D-cache port.
dcache #(.ADDR_WIDTH(ADDR_WIDTH), .CACHE_SIZE(DCACHE_SIZE))
D_Cache(
    .clk_i(clk_i),
    .rst_i(rst_i),

    .p_req_i(p_d_req && (mem_sel == 1)),
    .p_rw_i(data_rw && (mem_sel == 1)),
    .p_byte_enable_i(p_d_byte_enable),
    .p_addr_i(p_d_addr),
    .p_data_o(data_from_cache),
    .p_data_i(p_d_din),
    .p_ready_o(cache_d_ready),

    .m_addr_o(m_d_addr),
    .m_data_i(m_d_dout),
    .m_data_o(m_d_din),
    .m_strobe_o(m_d_strobe),
    .m_rw_o(m_d_rw),
    .m_ready_i(m_d_ready)
);

endmodule
