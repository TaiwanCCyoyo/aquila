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

    // Signal that stalls this Memory-WriteBack Pipeline Stage
    input  wire                    stall_i,

    // Processor pipeline flush signal.
    input  wire                    flush_i,

    output wire                     stall_for_data_fetch_o,

    // from Execute_Memory_Pipeline
    input  wire [DATA_WIDTH-1 : 0]  unaligned_data_i,       // from rs2
    input  wire [31: 0]             mem_addr_i,
    input  wire [1 : 0]             mem_input_sel_i,
    input  wire                     exe_we_i,
    input  wire                     exe_re_i,

    // to d-cache
    output wire [DATA_WIDTH-1 : 0]   data_o,           // data_write
    output wire [3: 0]               byte_write_sel_o,
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

    //exception from execute
    input  wire                     exp_vld_i,
    input  wire [3 : 0]             exp_cause_i,
    input  wire [31: 0]             exp_tval_i,
    input  wire [31: 0]             instruction_pc_i,
    input  wire                     instruction_pc_vld_i,


    //exception from mmu
    input  wire                     exp_from_mmu_vld_i,
    input  wire [3 : 0]             exp_from_mmu_cause_i,
    input  wire [31: 0]             exp_from_mmu_tval_i,

    //exception to writeback
    output wire                     exp_vld_o,
    output wire                     exp_isinterrupt_o,
    output wire [3 : 0]             exp_cause_o,
    output wire [31: 0]             exp_tval_o,
    output wire [31: 0]             instruction_pc_o,
    output wire                     instruction_pc_vld_o,

    //to Execute Execute_Memory_Pipeline
    input  wire                    regfile_we_i,
    input  wire [4 : 0]            rd_addr_i,
    input  wire [2 : 0]            regfile_input_sel_i,
    input  wire                    mem_load_ext_sel_i,
    input  wire [1 : 0]            mem_addr_alignment_i,
    input  wire [DATA_WIDTH-1 : 0] p_data_i,
    input  wire                    csr_we_i,
    input  wire [11: 0]            csr_we_addr_i,
    input  wire [DATA_WIDTH-1 : 0] csr_we_data_i,

    //to Writeback Memory_Writeback_Pipeline
    output wire [2: 0]             regfile_input_sel_o,
    output wire                    regfile_we_o,
    output wire [4: 0]             rd_addr_o,
    output wire                    mem_load_ext_sel_o,
    output wire [1: 0]             mem_addr_alignment_o,
    output wire [DATA_WIDTH-1 : 0] p_data_o,
    output wire                    csr_we_o,
    output wire [31: 0]            csr_we_addr_o,
    output wire [31: 0]            csr_we_data_o,
    output wire [DATA_WIDTH-1 : 0] mem_data_o,
    output wire [31: 0]            mip_update_o,

    // Interrupt requests.
    input  wire                    m_ext_irq_i, //Machine external interrupt
    input  wire                    m_tmr_irq_i, //Machine timer interrupt
    input  wire                    m_sft_irq_i, //Machine software interrupt
    input  wire                    s_ext_irq_i, //Supervisor external interrupt
    input  wire                    s_tmr_irq_i, //Supervisor timer interrupt
    input  wire                    s_sft_irq_i, //Supervisor software interrupt
    input  wire                    u_ext_irq_i, //User external interrupt
    input  wire                    u_tmr_irq_i, //User timer interrupt
    input  wire                    u_sft_irq_i, //User software interrupt

    //From CSR
    input  wire [ 3: 0]            mstatus_ie_i, //{MIE, WPRI, SIE, UIE}
    input  wire [31: 0]            mie_i

);
//=======================================================
// Parameter and Integer
//=======================================================
localparam d_IDLE = 0, d_WAIT = 1;

//=======================================================
// Wire and Reg 
//=======================================================
//-----------------------------------------------
// Memory access
//-----------------------------------------------
reg                     dS, dS_nxt;
reg  [DATA_WIDTH-1 : 0] data;           // data_write
reg  [3: 0]             byte_write_sel;

//-----------------------------------------------
// Exception
//-----------------------------------------------
reg          memory_alignment_exception;
reg          exp_vld;
reg          exp_isinterrupt;
reg  [3 : 0] exp_cause;
reg  [31: 0] exp_tval;

//-----------------------------------------------
// Memory data buffer when stall
//-----------------------------------------------
reg                     req_done;
reg                     buffer_exp_vld;
reg [3 : 0]             buffer_exp_cause;
reg [31: 0]             buffer_exp_tval;
reg [DATA_WIDTH-1 : 0]  buffer_data;

//-----------------------------------------------
// Pipeline reg 
//-----------------------------------------------
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
reg                    instruction_pc_vld_r;
reg                    csr_we_r;
reg [31: 0]            csr_we_addr_r;
reg [31: 0]            csr_we_data_r;
reg [DATA_WIDTH-1 : 0] mem_data_r;

reg [31: 0]            mip_update_r;
reg                    exp_isinterrupt_r;


//=======================================================
// User Logic                         
//=======================================================
//-----------------------------------------------
// Memory access FSM
//-----------------------------------------------
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
// ===============================================================================
//  Memory data buffer when stall

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

// ===============================================================================
//  Exception & Interrupt
//  ext > tmr > sft
always@(*)
begin
    if(m_ext_irq_i && mstatus_ie_i[3] && mie_i[11])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd11;
        exp_tval        = 'd0;
    end
    else if(m_tmr_irq_i && mstatus_ie_i[3] && mie_i[7])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd7;
        exp_tval        = 'd0;
    end
    else if(m_sft_irq_i && mstatus_ie_i[3] && mie_i[3])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd3;
        exp_tval        = 'd0;
    end
    else if(s_ext_irq_i && mstatus_ie_i[1] && mie_i[9])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd9;
        exp_tval        = 'd0;
    end
    else if(s_tmr_irq_i && mstatus_ie_i[1] && mie_i[5])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd5;
        exp_tval        = 'd0;
    end
    else if(s_sft_irq_i && mstatus_ie_i[1] && mie_i[1])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd1;
        exp_tval        = 'd0;
    end
     else if(u_ext_irq_i && mstatus_ie_i[0] && mie_i[8])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd8;
        exp_tval        = 'd0;
    end
    else if(u_tmr_irq_i && mstatus_ie_i[0] && mie_i[4])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd4;
        exp_tval        = 'd0;
    end
    else if(u_sft_irq_i && mstatus_ie_i[0] && mie_i[0])
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b1;
        exp_cause       = 'd0;
        exp_tval        = 'd0;
    end
    else if(memory_alignment_exception && (exe_we_i || exe_re_i))
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b0;
        exp_cause       = 'd11;
        exp_tval        = 'd0;
    end
    else if(exp_from_mmu_vld_i)
    begin
        exp_vld         = 1'b1;
        exp_isinterrupt = 1'b0;
        exp_cause       = exp_from_mmu_cause_i;
        exp_tval        = exp_from_mmu_tval_i;
    end
    else if(req_done)
    begin
        exp_vld         = buffer_exp_vld;
        exp_isinterrupt = 1'b0;
        exp_cause       = buffer_exp_cause;
        exp_tval        = buffer_exp_tval;
    end
    else
    begin
        exp_vld         = exp_vld_i;
        exp_isinterrupt = 1'b0;
        exp_cause       = exp_cause_i;
        exp_tval        = exp_tval_i;
    end
end

//-----------------------------------------------
// interrupt update
//-----------------------------------------------


// ===============================================================================
//  Memory_alignment_exception

// store
always @(*)
begin
    case (mem_addr_i[1:0])
        2'b00:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin   // sb
                    data = {24'b0, unaligned_data_i[7: 0]};
                    byte_write_sel = 4'b0001;
                    memory_alignment_exception = 0;
                end
                2'b01:
                begin   // sh
                    data = {16'b0, unaligned_data_i[15: 0]};
                    byte_write_sel = 4'b0011;
                    memory_alignment_exception = 0;
                end
                2'b10:
                begin   // sw
                    data = unaligned_data_i;
                    byte_write_sel = 4'b1111;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data = 0;
                    byte_write_sel = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        2'b01:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin   // sb
                    data = {16'b0, unaligned_data_i[7: 0], 8'b0};
                    byte_write_sel = 4'b0010;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data = 0;
                    byte_write_sel = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        2'b10:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin
                    data = {8'b0, unaligned_data_i[7: 0], 16'b0};
                    byte_write_sel = 4'b0100;
                    memory_alignment_exception = 0;
                end
                2'b01:
                begin
                    data = {unaligned_data_i[15: 0], 16'b0};
                    byte_write_sel = 4'b1100;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data = 0;
                    byte_write_sel = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        2'b11:
        begin
            case (mem_input_sel_i)
                2'b00:
                begin
                    data = {unaligned_data_i[7: 0], 24'b0};
                    byte_write_sel = 4'b1000;
                    memory_alignment_exception = 0;
                end
                default:
                begin
                    data = 0;
                    byte_write_sel = 4'b0000;
                    memory_alignment_exception = 1;
                end
            endcase
        end
        default:
        begin
            data = 0;
            byte_write_sel = 4'b0000;
            memory_alignment_exception = 0;
        end
    endcase
end

// ===============================================================================
//  Pipeline register for the Memory stage.
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
        instruction_pc_vld_r <= 0;
        mem_data_r           <= 32'b0;
        exp_isinterrupt_r    <= 0;
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
        instruction_pc_vld_r <= instruction_pc_vld_r;
        mem_data_r           <= mem_data_r;
        exp_isinterrupt_r    <= exp_isinterrupt_r;
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
        instruction_pc_vld_r <= 0;
        mem_data_r           <= 32'b0;
        exp_isinterrupt_r    <= 0;
    end
    else if(exp_vld)
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
        exp_vld_r            <= exp_vld;
        exp_cause_r          <= exp_cause;
        exp_tval_r           <= exp_tval;
        instruction_pc_r     <= instruction_pc_i;
        instruction_pc_vld_r <= instruction_pc_vld_i;
        mem_data_r           <= 32'b0;
        exp_isinterrupt_r    <= exp_isinterrupt;
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
        exp_vld_r            <= exp_vld;
        exp_cause_r          <= exp_cause;
        exp_tval_r           <= exp_tval;
        instruction_pc_r     <= instruction_pc_i;
        instruction_pc_vld_r <= instruction_pc_vld_i;
        mem_data_r           <= (req_done)?buffer_data:d_rtrn_data_i;
        exp_isinterrupt_r    <= exp_isinterrupt;
    end
end

//-----------------------------------------------
// mip update
//-----------------------------------------------
// --------------------------------------------------------------------------------------------
// | WPRI | MEIP | WPRI | SEIP | UEIP | MTIP | WPRI | STIP | UTIP | MSIP | WPRI | SSIP | USIP |
// --------------------------------------------------------------------------------------------
// |31  12|  11  |  10  |  9   |  8   |  7   |  6   |  5   |  4   |  3   |  2   |  1   |  0   |
// --------------------------------------------------------------------------------------------
always@(posedge clk_i)
begin
    if(rst_i)
    begin
        mip_update_r <= 0;
    end
    else
    begin
        mip_update_r <= {20'b0, {m_ext_irq_i, 1'b0, s_ext_irq_i, u_ext_irq_i}, 
                                {m_tmr_irq_i, 1'b0, s_tmr_irq_i, u_tmr_irq_i}, 
                                {m_sft_irq_i, 1'b0, s_sft_irq_i, u_sft_irq_i}} & mie_i;
    end
end


//=======================================================
// Output signals interface                       
//=======================================================
//-----------------------------------------------
// Memory access
//-----------------------------------------------
assign req_vld_o              = (dS_nxt == d_WAIT);      
assign req_vaddr_o            = mem_addr_i;
assign req_rw_o               = exe_we_i;
assign data_o                 = data;
assign byte_write_sel_o       = byte_write_sel;

assign stall_for_data_fetch_o = (dS_nxt == d_WAIT);

//-----------------------------------------------
// Pipeline register to Writeback
//-----------------------------------------------
assign regfile_input_sel_o  = regfile_input_sel_r;
assign regfile_we_o         = regfile_we_r;
assign rd_addr_o            = rd_addr_r;
assign mem_load_ext_sel_o   = mem_load_ext_sel_r;
assign mem_addr_alignment_o = mem_addr_alignment_r;
assign p_data_o             = p_data_r;
assign exp_vld_o            = exp_vld_r;
assign exp_cause_o          = exp_cause_r;
assign exp_tval_o           = exp_tval_r;
assign sys_jump_o           = sys_jump_r;
assign sys_jump_csr_addr_o  = sys_jump_csr_addr_r;
assign instruction_pc_o     = instruction_pc_r;
assign instruction_pc_vld_o = instruction_pc_vld_r;
assign csr_we_o             = csr_we_r;
assign csr_we_addr_o        = csr_we_addr_r;
assign csr_we_data_o        = csr_we_data_r;
assign mem_data_o           = mem_data_r;
assign mip_update_o         = mip_update_r;
assign exp_isinterrupt_o    = exp_isinterrupt_r;



endmodule   // memory_access
