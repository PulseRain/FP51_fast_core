/*
###############################################################################
# Copyright (c) 2017, PulseRain Technology LLC 
#
# This program is distributed under a dual license: an open source license, 
# and a commercial license. 
# 
# The open source license under which this program is distributed is the 
# GNU Public License version 3 (GPLv3).
#
# And for those who want to use this program in ways that are incompatible
# with the GPLv3, PulseRain Technology LLC offers commercial license instead.
# Please contact PulseRain Technology LLC (www.pulserain.com) for more detail.
#
###############################################################################
*/

//=============================================================================
// Remarks:
//    pipeline stage after control (instruction decoder). This stage is needed
// before interacting with BRAM. Otherwise timing will be affected negatively.
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none


//-----------------------------------------------------------------------------
// fast_core_post_control
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      
//
// See Also:
//    
//-----------------------------------------------------------------------------

module fast_core_post_control (
        
        
        //========== INPUT ==========
        
        input   wire                            clk,
        input   wire                            reset_n,
        
        input   wire unsigned [23 : 0]              IR,
        input   wire unsigned [PC_BITWIDTH - 1 : 0] PC,
        
        input   wire unsigned [7 : 0]           R0,
        input   wire unsigned [7 : 0]           R1,
        input   wire unsigned [7 : 0]           XPAGE,
        
        input   wire                                reg_jump_active,
        input   wire                                ctl_interruptable,
        input   wire                                int_gen_flag,
    
        input   wire  unsigned [1 : 0]              size_of_instruction,
        
        input   wire  unsigned [ADDR_WIDTH - 1 : 0]     DPTR,
        
        input   wire  unsigned [23 : 0]                 preALU_reg_IR,
        input   wire                                    preALU_reg_ctl_DPTR_load_data,
        input   wire                                    preALU_reg_ctl_DPTR_inc,
        
        input   wire  unsigned [23 : 0]                 ALU_reg_IR,
        input   wire                                    ALU_reg_ctl_DPTR_load_data,
        input   wire                                    ALU_reg_ctl_DPTR_inc,
                    
        input   wire  unsigned [ADDR_WIDTH - 1 : 0]     SP,
        input   wire  unsigned [SP_INC_BITS - 1 : 0]    preALU_reg_ctl_stack_move_amount,
        input   wire  unsigned [SP_INC_BITS - 1 : 0]    ALU_reg_ctl_stack_move_amount,
                
        input   wire                                preALU_SP_write_lock,
        input   wire                                preALU_DPTR_write_lock,
        
        input   wire                                ALU_SP_write_lock,
        input   wire                                ALU_DPTR_write_lock,
        
        input   wire                                ctl_DPTR_load_data,
        input   wire                                ctl_DPTR_inc,
        input   wire                                ctl_DPTR_read,
        input   wire                                ctl_DPTR_write,
                
        input   wire  unsigned [SP_INC_BITS - 1 : 0]    ctl_stack_move_amount,
        input   wire                                    ctl_stack_read,
        input   wire                                    ctl_stack_write,
        
        input   wire                                    ctl_indirect_read,
        
        input   wire unsigned [7 : 0]           ALU_adjust_a,
        input   wire unsigned [7 : 0]           ALU_adjust_b,
        input   wire unsigned [1 : 0]           active_reg_bank_index,
                
        input   wire                            ctl_pipeline_hold,
        input   wire                            ctl_pipeline_resume,
        
        input   wire                            preALU_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   preALU_IR,
        input   wire                            ALU_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   ALU_IR,
        
        input   wire    [ALU_NUM_OF_OPERATIONS - 1 : 0] ctl_ALU_cmd,
        
        input   wire                            ctl_xread_enable,
        input   wire                            ctl_xwrite_enable,
        
        input   wire                            ctl_write_enable,
        input   wire                            ctl_indirect_write,
        input   wire  unsigned [7 : 0]          data_wr_address,
        input   wire  unsigned [7 : 0]          data_rd_address,
        
        input   wire  unsigned [15 : 0]         xdata_wr_address,
        input   wire  unsigned [15 : 0]         xdata_rd_address,
        
        input   wire                            ctl_ALU_active,
        input   wire                            ctl_ALU_adjust_a,
        input   wire                            ctl_ALU_adjust_b,
        input   wire                            ctl_ALU_data_a,
        input   wire                            ctl_ALU_data_b,
        input   wire                            ctl_PSW_flag_update,
        input   wire                            ctl_load_A,
        
        
        //========== OUTPUT ==========
    
        output logic  [ALU_NUM_OF_OPERATIONS - 1 : 0]   reg_ctl_ALU_cmd,
        
        output logic   unsigned [15 : 0]        reg_data_wr_address,
        output logic   unsigned [15 : 0]        reg_data_rd_address,
        
        output logic                            reg_ctl_indirect_write,
        output logic                            reg_ctl_PSW_flag_update,
        output logic                            reg_ctl_load_A,
        output logic                            reg_ctl_write_enable,
                
        output logic unsigned [7 : 0]           reg_ALU_adjust_a,
        output logic unsigned [7 : 0]           reg_ALU_adjust_b,
                
        output logic                            reg_ctl_ALU_adjust_a,
        output logic                            reg_ctl_ALU_adjust_b,
        
        output logic                            reg_ctl_ALU_data_a,
        output logic                            reg_ctl_ALU_data_b,
        
        output logic                            reg_ctl_xread_enable,
        output logic                            reg_ctl_xwrite_enable,
        
        output logic                            reg_ctl_indirect_read,
                        
        output logic  unsigned [23 : 0]                 reg_IR,
        output logic  unsigned [PC_BITWIDTH - 1 : 0]    reg_PC,
        output logic  unsigned [1 : 0]                  reg_size_of_instruction,
        output wire                                     ctl_preALU_active_out,
        output wire                                     indirect_R0_R1_access_flag,
        output logic                                    reg_ctl_ALU_bit_mask_a,
        output logic  unsigned [BIT_SHIFT_WIDTH - 1 : 0] reg_bit_mask_shift,
        output logic                                     reg_ctl_bit_mask_invert,
        output logic                                     ctl_bit_load,
        output logic                                     ctl_inv_bit_load,
        output logic                                     ctl_CY_update,
        
        output logic                                     reg_ctl_DPTR_load_data,
        output logic                                     reg_ctl_DPTR_inc,
        output logic                                     reg_ctl_DPTR_read,
        output logic                                     reg_ctl_DPTR_write,
        output logic  unsigned [SP_INC_BITS - 1 : 0]     reg_ctl_stack_move_amount,
        output logic                                     reg_ctl_stack_read,
        output logic                                     reg_ctl_stack_write,
        output logic                                     pipeline_stall,
        output logic                                     reg_ctl_interruptable,
        
        input wire                                       debug_stall,
        input wire                                       debug_data_read,
        input wire                                       debug_rd_indirect1_direct0,
        input wire unsigned [15 : 0]                     debug_data_read_addr,
        input wire                                       debug_data_read_restore
        
        
);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        logic                                   running;
        logic                                   reg_ctl_ALU_active;
        logic                                   ctl_ALU_bit_mask_a;
        logic unsigned [ADDR_WIDTH - 1 : 0]     bit_data_rd_addr, bit_data_wr_addr;
        logic unsigned [ALU_NUM_OF_OPERATIONS - 1 : 0]  bit_ctl_ALU_cmd;
        logic                                           ctl_bit_mask_invert;
        logic unsigned [BIT_SHIFT_WIDTH - 1 : 0]        bit_mask_shift;
        
        logic  unsigned [ADDR_WIDTH - 1 : 0]            bit_addr_2_RAM_addr;
        wire   unsigned [BIT_SHIFT_WIDTH - 1 : 0]       bit_addr_shift_amount;
        
        logic                                           ctl_bit_load_i, ctl_inv_bit_load_i;
        logic                                           ctl_CY_update_i;
        
        wire                                            SP_write_lock, DPTR_write_lock;
        
        wire    unsigned [SP_INC_BITS - 1 : 0]          total_stack_move_amount, partial_stack_move_amount;
        
        wire                                            indirect_conflict;
        logic   unsigned [1 : 0]                        indirect_conflict_sr;
        
        wire    unsigned [7 : 0]                        indirect_R, reg_indirect_R;
        
        logic   unsigned [15 : 0]                       reg_data_rd_address_save;
        logic                                           reg_ctl_indirect_read_save;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // registers
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        assign total_stack_move_amount = reg_ctl_stack_move_amount + 
           preALU_reg_ctl_stack_move_amount + ALU_reg_ctl_stack_move_amount;
        
        assign partial_stack_move_amount = 
                preALU_reg_ctl_stack_move_amount + ALU_reg_ctl_stack_move_amount;
        
        assign indirect_conflict = ( ( (indirect_R0_R1_access_flag && (IR[16] == reg_IR[16])) ||
                                       (preALU_indirect_R0_R1_access_flag && (IR[16] == preALU_IR[16])) ||
                                       (ALU_indirect_R0_R1_access_flag && (IR[16] == ALU_IR[16]))) & ctl_ALU_active) ? 1'b1 : 1'b0;
        
        assign indirect_R  = (IR[16]) ? R1 : R0;
        assign reg_indirect_R = (reg_IR[16]) ? R1 : R0;
        
        always_ff @(posedge clk, negedge reset_n) begin : reg_proc
            if (!reset_n) begin
                reg_ctl_ALU_active      <= 0;
                reg_ctl_ALU_cmd         <= 0;
                reg_ctl_indirect_write  <= 0;
                reg_ctl_PSW_flag_update <= 0;
                reg_ctl_load_A          <= 0;
                reg_data_wr_address     <= 0;
                reg_IR                  <= 0;
                reg_PC                  <= 0;
                reg_data_wr_address     <= 0;
                reg_data_rd_address     <= 0;
                reg_ctl_ALU_active      <= 0;
                    
                reg_ALU_adjust_a        <= 0;
                reg_ALU_adjust_b        <= 0;
                    
                reg_ctl_ALU_data_a      <= 0;
                reg_ctl_ALU_data_b      <= 0;
                    
                reg_ctl_ALU_adjust_a    <= 0;
                reg_ctl_ALU_adjust_b    <= 0;
                reg_ctl_write_enable    <= 0;
                
                reg_ctl_xread_enable    <= 0;
                reg_ctl_xwrite_enable   <= 0;
        
                reg_ctl_ALU_bit_mask_a  <= 0;
                
                reg_bit_mask_shift      <= 0;
                reg_ctl_bit_mask_invert <= 0;
                
                reg_ctl_DPTR_load_data     <= 0;
                reg_ctl_DPTR_inc           <= 0;
                reg_ctl_DPTR_read          <= 0;
                reg_ctl_DPTR_write         <= 0;
                reg_ctl_stack_move_amount  <= 0;
                reg_ctl_stack_read         <= 0;
                reg_ctl_stack_write        <= 0;
                
                reg_ctl_indirect_read      <= 0;
                reg_ctl_interruptable      <= 0;
                reg_size_of_instruction    <= 0;
                
                indirect_conflict_sr       <= 0;
                
                reg_data_rd_address_save   <= 0;
                reg_ctl_indirect_read_save <= 0;
            end else begin
                
                indirect_conflict_sr <= {indirect_conflict_sr [$high(indirect_conflict_sr) - 1 : 0], indirect_conflict};
                
                if (debug_data_read) begin
                    reg_data_rd_address_save   <= reg_data_rd_address;
                    reg_ctl_indirect_read_save <= reg_ctl_indirect_read;
                    
                    reg_data_rd_address        <= debug_data_read_addr;
                    reg_ctl_indirect_read      <= debug_rd_indirect1_direct0;

                end else if (debug_data_read_restore) begin
                    reg_data_rd_address     <= reg_data_rd_address_save;
                    reg_ctl_indirect_read   <= reg_ctl_indirect_read_save;
                end else if (ctl_ALU_active & running & (~reg_jump_active) & (~int_gen_flag)) begin
                    reg_ctl_ALU_cmd         <= ctl_ALU_cmd | bit_ctl_ALU_cmd;
                    reg_ctl_indirect_write  <= ctl_indirect_write;
                    reg_ctl_PSW_flag_update <= ctl_PSW_flag_update;
                    reg_ctl_load_A          <= ctl_load_A;
                    
                    reg_IR                  <= IR;
                    reg_PC                  <= PC;
                    reg_size_of_instruction <= size_of_instruction;
                    
                    
                    case (1'b1) // synthesis parallel_case
                        ctl_ALU_bit_mask_a : begin
                            reg_data_wr_address <= bit_data_wr_addr;
                        end
                        
                        ctl_xwrite_enable | ctl_indirect_write: begin
                            if (ctl_indirect_write) begin
                                if (ctl_xwrite_enable) begin
                                    reg_data_wr_address <= {XPAGE, indirect_R} + ($size(reg_data_wr_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                                end else begin
                                    reg_data_wr_address <= {8'd0, indirect_R};  
                                end
                            end else begin
                                reg_data_wr_address <= xdata_wr_address + ($size(reg_data_wr_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                            end
                        end
                        
                        default : begin
                            reg_data_wr_address <= {8'd0, data_wr_address}; 
                        end
                        
                    endcase
                                        
                    case (1'b1) // synthesis parallel_case 
                        
                        ctl_DPTR_read  : begin
                            if (reg_ctl_DPTR_load_data) begin
                                reg_data_rd_address <= reg_IR [15 : 0] + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                            end else if (preALU_reg_ctl_DPTR_load_data) begin
                                reg_data_rd_address <= preALU_reg_IR [15 : 0] + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES) + {15'd0, reg_ctl_DPTR_inc};
                            end else if (ALU_reg_ctl_DPTR_load_data) begin
                                reg_data_rd_address <= ALU_reg_IR [15 : 0] + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES) + {15'd0, preALU_reg_ctl_DPTR_inc} + {15'd0, reg_ctl_DPTR_inc}; 
                            end else begin
                                reg_data_rd_address <= (DPTR + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES) + {15'd0, ALU_reg_ctl_DPTR_inc}) + ({15'd0, preALU_reg_ctl_DPTR_inc} + {15'd0, reg_ctl_DPTR_inc});
                            end
                        end
                                                
                        ctl_stack_read  : begin
                            reg_data_rd_address <= SP + 
                                {{(ADDR_WIDTH - SP_INC_BITS){total_stack_move_amount[$high(total_stack_move_amount)]}}, total_stack_move_amount};
                        end
                        
                        ctl_ALU_bit_mask_a : begin
                            reg_data_rd_address <= bit_data_rd_addr;
                        end
                        
                        ctl_xread_enable | ctl_indirect_read : begin
                            if (ctl_indirect_read) begin
                                if (ctl_xread_enable) begin
                                    reg_data_rd_address  <= {XPAGE, indirect_R} + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                                end else begin
                                    reg_data_rd_address  <= {8'd0, indirect_R};
                                end
                            end else begin
                                reg_data_rd_address  <= xdata_rd_address + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                            end
                        end
                        
                        default : begin
                            reg_data_rd_address  <= {8'd0, data_rd_address};    
                        end
                    
                    endcase
                    
                    reg_ctl_ALU_active      <= 1;
                        
                    reg_ALU_adjust_a        <= ALU_adjust_a;
                    reg_ALU_adjust_b        <= ALU_adjust_b;
                        
                    reg_ctl_ALU_data_a      <= ctl_ALU_data_a;
                    reg_ctl_ALU_data_b      <= ctl_ALU_data_b;
                        
                    reg_ctl_ALU_adjust_a    <= ctl_ALU_adjust_a;
                    reg_ctl_ALU_adjust_b    <= ctl_ALU_adjust_b;
                    reg_ctl_write_enable    <= (ctl_write_enable | ctl_xwrite_enable ) & (~ctl_CY_update_i);
                    
                    reg_ctl_xread_enable    <= ctl_xread_enable;
                    reg_ctl_xwrite_enable   <= ctl_xwrite_enable;
                
                    reg_ctl_ALU_bit_mask_a  <= ctl_ALU_bit_mask_a;
                    reg_bit_mask_shift      <= bit_mask_shift;
                    reg_ctl_bit_mask_invert <= ctl_bit_mask_invert;
                    
                    reg_ctl_DPTR_load_data  <= ctl_DPTR_load_data;
                    reg_ctl_DPTR_inc        <= ctl_DPTR_inc;
                    reg_ctl_DPTR_read       <= ctl_DPTR_read;
                    reg_ctl_DPTR_write      <= ctl_DPTR_write;
                    reg_ctl_stack_move_amount  <= ctl_stack_move_amount;
                    reg_ctl_stack_read      <= ctl_stack_read;
                    reg_ctl_stack_write     <= ctl_stack_write;
                    
                    reg_ctl_indirect_read   <= ctl_indirect_read;
                    reg_ctl_interruptable   <= ctl_interruptable;
                    
                end else begin
                    reg_ctl_ALU_active      <= (pipeline_stall) & (~reg_jump_active);
                    
                    if (reg_ctl_stack_read) begin
                        reg_data_rd_address <= SP + 
                                {{(ADDR_WIDTH - SP_INC_BITS){partial_stack_move_amount[$high(partial_stack_move_amount)]}}, partial_stack_move_amount};
                    end else if (reg_ctl_DPTR_read) begin
                        
                        if (preALU_reg_ctl_DPTR_load_data) begin
                            reg_data_rd_address <= preALU_reg_IR [15 : 0] + {15'd0, reg_ctl_DPTR_inc} + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                        end else if (ALU_reg_ctl_DPTR_load_data) begin
                            reg_data_rd_address <= ALU_reg_IR [15 : 0] + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES) + {15'd0, preALU_reg_ctl_DPTR_inc} + {15'd0, reg_ctl_DPTR_inc}; 
                        end else begin
                            reg_data_rd_address <= (DPTR + ($size(reg_data_rd_address))'(XRAM_ADDR_OFFSET_IN_BYTES) + {15'd0, ALU_reg_ctl_DPTR_inc}) + ({15'd0, preALU_reg_ctl_DPTR_inc} + {15'd0, reg_ctl_DPTR_inc});
                        end
                    end else if (reg_ctl_indirect_read) begin
                        reg_data_rd_address  <= {7'd0, reg_ctl_xread_enable, reg_indirect_R};
                    end 
                    
                    if (reg_ctl_indirect_write) begin
                        reg_data_wr_address  <= {7'd0, reg_ctl_xwrite_enable, reg_indirect_R};
                    end
                    
                    if (reg_jump_active) begin
                        reg_IR <= INST_NOP;
                    end 
                    
                    reg_ctl_DPTR_load_data    <= 0;
                    reg_ctl_DPTR_inc          <= 0;
                    
                    if (!pipeline_stall) begin
                        reg_ctl_stack_move_amount <= 0;
                    end
                end
            end
        end : reg_proc
    
        assign ctl_preALU_active_out = reg_ctl_ALU_active |  pipeline_stall;
        
        assign indirect_R0_R1_access_flag = ((!reg_data_wr_address [15 : 3]) 
            && (reg_data_wr_address[2:1] == active_reg_bank_index) && reg_ctl_ALU_active) ? 1'b1 : 1'b0; 
        
        
        assign SP_write_lock = (((reg_data_wr_address == {8'd0, SPL_ADDR}) || (reg_data_wr_address == {8'd0, SPH_ADDR})) 
                   && reg_ctl_ALU_active) ? 1'b1 : 1'b0;
        assign DPTR_write_lock = (((reg_data_wr_address == {8'd0, DPL_ADDR}) || (reg_data_wr_address == {8'd0, DPH_ADDR}))
                   && reg_ctl_ALU_active) ? 1'b1 : 1'b0;
                
        assign bit_addr_shift_amount = IR [10 : 8];
        
        always_comb begin
            
            bit_addr_2_RAM_addr [15 : 8] = 0;
            
            if (!IR [15]) begin
                bit_addr_2_RAM_addr [7 : 4] = 4'h2;
                bit_addr_2_RAM_addr [3 : 0] = IR [14 : 11];
            end else begin
                bit_addr_2_RAM_addr [7 : 0] = {IR [15 : 11], 3'h0};
            end
        end
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                ctl_bit_load <= 0;
                ctl_inv_bit_load <= 0;
                ctl_CY_update <= 0;
            end else begin
                ctl_bit_load <= ctl_bit_load_i;
                ctl_inv_bit_load <= ctl_inv_bit_load_i;
                ctl_CY_update <= ctl_CY_update_i;
            end
        end
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Pipeline Stall
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                pipeline_stall <= 0;
            end else begin
                if ( ((SP_write_lock | preALU_SP_write_lock |  ALU_SP_write_lock) & (ctl_stack_read | reg_ctl_stack_read | ctl_stack_write | reg_ctl_stack_write)) |
                     ((DPTR_write_lock | preALU_DPTR_write_lock | ALU_DPTR_write_lock) & (ctl_DPTR_read | reg_ctl_DPTR_read | ctl_DPTR_write)) ) begin
                    pipeline_stall <= 1'b1;
                end else if ( (indirect_conflict | (|indirect_conflict_sr)) & ((ctl_indirect_read & ctl_ALU_active) | (reg_ctl_indirect_read & (~ctl_ALU_active)) ) ) begin 
                    pipeline_stall <= 1'b1;
                end else begin
                    pipeline_stall <= 0;
                end 
            end
        end
        
        
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_RUN, S_HOLD} states = S_RUN;
                    
        localparam FSM_NUM_OF_STATES = states.num();
        logic [FSM_NUM_OF_STATES - 1:0] current_state = 0, next_state = 0;
                    
        // Declare states
        always_ff @(posedge clk, negedge reset_n) begin : state_machine_reg
            if (!reset_n) begin
                current_state <= 0;
            end else begin
                current_state <= next_state;
            end
        end : state_machine_reg
                
        // state cast for debug, one-hot translation, enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "states" variable
                
        // synthesis translate_off
        ///////////////////////////////////////////////////////////////////////
        always_comb begin : state_cast_for_debug
            for (int i = 0; i < FSM_NUM_OF_STATES; ++i) begin
                if (current_state[i]) begin
                    $cast(states, i);
                end
            end
        end : state_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on   
                
        // FSM main body
        always_comb begin : state_machine_comb
    
            next_state = 0;
                
            running = 1;
            
            ctl_ALU_bit_mask_a = 0;
            
            bit_data_rd_addr = 0;
            bit_data_wr_addr = 0;
        
            bit_ctl_ALU_cmd = 0;
            
            ctl_bit_mask_invert = 0;
            bit_mask_shift = 0;
            
            ctl_bit_load_i = 0;
            ctl_inv_bit_load_i = 0;
            
            ctl_CY_update_i = 0;
            
            case (1'b1) // synthesis parallel_case 
                    
                current_state[S_RUN]: begin
                        
                    if (ctl_pipeline_hold) begin
                        running = 0;
                        next_state [S_HOLD] = 1;
                    end else begin
                        next_state [S_RUN] = 1; 
                    end
                        
                    case (IR[23 : 16]) // synthesis parallel_case 
                        INST_CLR_C : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            
                            bit_data_rd_addr = PSW_ADDR;
                            bit_data_wr_addr = PSW_ADDR;
                            bit_ctl_ALU_cmd [OP_ALU_AND] = 1;
                            ctl_bit_mask_invert = 1'b1;
                            bit_mask_shift = ($size(bit_mask_shift))'(PSW_CY_INDEX);
                        end
                        
                        INST_SETB_C : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            
                            bit_data_rd_addr = PSW_ADDR;
                            bit_data_wr_addr = PSW_ADDR;
                            bit_ctl_ALU_cmd [OP_ALU_OR] = 1;
                            bit_mask_shift = ($size(bit_mask_shift))'(PSW_CY_INDEX);
                        end
                        
                        INST_CPL_C : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            
                            bit_data_rd_addr = PSW_ADDR;
                            bit_data_wr_addr = PSW_ADDR;
                            bit_ctl_ALU_cmd [OP_ALU_XOR] = 1;
                            bit_mask_shift = ($size(bit_mask_shift))'(PSW_CY_INDEX);
                        end
                        
                        INST_CLR_BIT : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_data_wr_addr = bit_addr_2_RAM_addr;
                            bit_ctl_ALU_cmd [OP_ALU_AND] = 1;
                            ctl_bit_mask_invert = 1'b1;
                            bit_mask_shift = bit_addr_shift_amount;
                        end
                        
                        INST_SETB_BIT : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_data_wr_addr = bit_addr_2_RAM_addr;
                            bit_ctl_ALU_cmd [OP_ALU_OR] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                        end
                        
                        INST_CPL_BIT : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_data_wr_addr = bit_addr_2_RAM_addr;
                            bit_ctl_ALU_cmd [OP_ALU_XOR] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                        end
                        
                        INST_ANL_C_BIT : begin
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            
                            bit_ctl_ALU_cmd [OP_ALU_AND] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_CY_update_i = 1'b1;
                            ctl_bit_load_i = 1'b1;
                        end
                        
                        INST_ANL_C_NBIT : begin
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            
                            bit_ctl_ALU_cmd [OP_ALU_AND] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_CY_update_i = 1'b1;
                            ctl_inv_bit_load_i = 1'b1;
                        end
                        
                        INST_ORL_C_BIT : begin
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            
                            bit_ctl_ALU_cmd [OP_ALU_OR] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_CY_update_i = 1'b1;
                            ctl_bit_load_i = 1'b1;
                        end
                        
                        INST_ORL_C_NBIT : begin
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            
                            bit_ctl_ALU_cmd [OP_ALU_OR] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_CY_update_i = 1'b1;
                            ctl_inv_bit_load_i = 1'b1;
                        end
                        
                        INST_MOV_C_BIT : begin
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            
                            bit_ctl_ALU_cmd [OP_ALU_XOR] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_CY_update_i = 1'b1;
                            ctl_bit_load_i = 1'b1;
                        end
                        
                        INST_MOV_BIT_C : begin
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_data_wr_addr = bit_addr_2_RAM_addr;
                            
                            bit_ctl_ALU_cmd [OP_ALU_MOV_BIT_C] = 1;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_bit_load_i = 1'b1;
                        end
                        
                        INST_JB : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_bit_load_i = 1'b1;
                        end
                        
                        INST_JNB : begin
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_inv_bit_load_i = 1'b1;
                        end
                        
                        INST_JBC : begin
                            if (PC == reg_PC) begin
                                bit_data_wr_addr = bit_addr_2_RAM_addr;
                                bit_ctl_ALU_cmd [OP_ALU_AND] = 1;
                                ctl_bit_mask_invert = 1'b1;
                            end
                            
                            ctl_ALU_bit_mask_a = 1'b1;
                            bit_data_rd_addr = bit_addr_2_RAM_addr;
                            bit_mask_shift = bit_addr_shift_amount;
                            ctl_bit_load_i = 1'b1;
                        end                 
                        
                        
                        default : begin
                            
                        end
                            
                    endcase
                
                end
                    
                current_state [S_HOLD] : begin
                    running = 0;
        
                    if (ctl_pipeline_resume) begin
                        next_state [S_RUN] = 1;
                    end else begin
                        next_state [S_HOLD] = 1;
                    end
                end
                            
                default: begin
                    next_state[S_RUN] = 1'b1;
                end
                    
            endcase
                  
        end : state_machine_comb
                
            
endmodule : fast_core_post_control

`default_nettype wire
