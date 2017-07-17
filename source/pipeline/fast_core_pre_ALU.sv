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
//    pipeline stage before ALU. This stage is needed to hold the inputs to
// ALU. 
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// fast_core_pre_ALU
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

module fast_core_pre_ALU (
        
         //========== INPUT ==========
        input   wire                            clk,
        input   wire                            reset_n,
        
        input   wire                                int_gen_flag,
        input   wire unsigned [23 : 0]              IR,
        input   wire unsigned [PC_BITWIDTH - 1 : 0] PC,
        input   wire  unsigned [1 : 0]              size_of_instruction,
        
        input   wire                                pipeline_stall,
        input   wire                                reg_jump_active,
        input   wire                                ctl_interruptable,
        
        input   wire                                ctl_DPTR_load_data,
        input   wire                                ctl_DPTR_inc,
        input   wire                                ctl_DPTR_read,
        input   wire                                ctl_DPTR_write,
                
        input   wire  unsigned [SP_INC_BITS - 1 : 0]    ctl_stack_move_amount,
        input   wire                                    ctl_stack_read,
        input   wire                                    ctl_stack_write,
                
        input   wire unsigned [7 : 0]           ALU_adjust_a,
        input   wire unsigned [7 : 0]           ALU_adjust_b,
        input   wire unsigned [1 : 0]           active_reg_bank_index,
        
        input   wire                            ctl_pipeline_hold,
        input   wire                            ctl_pipeline_resume,
        
        input   wire                            ctl_bit_load,
        input   wire                            ctl_inv_bit_load,
        input   wire                            ctl_CY_update,
        
        input   wire                            ctl_ALU_bit_mask_a,
        input   wire                            ctl_bit_mask_invert,
        input   wire unsigned [BIT_SHIFT_WIDTH - 1 : 0] bit_mask_shift,
                
        input   wire    [ALU_NUM_OF_OPERATIONS - 1 : 0] ctl_ALU_cmd,
        
        input   wire                            ctl_xread_enable,
        input   wire                            ctl_xwrite_enable,
        
        input   wire                            ctl_indirect_read,
        
        input   wire                            ctl_write_enable,
        input   wire                            ctl_indirect_write,
        input   wire  unsigned [15 : 0]         data_wr_address,
        input   wire  unsigned [15 : 0]         data_rd_address,
        
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
                
        output logic                            ctl_RL_A,
        output logic                            ctl_RLC_A,
        output logic                            ctl_RR_A,
        output logic                            ctl_RRC_A,
        
        output logic  unsigned [23 : 0]                 reg_IR,
        output logic  unsigned [PC_BITWIDTH - 1 : 0]    reg_PC,
        output logic  unsigned [1 : 0]                  reg_size_of_instruction,
        output wire                                     ctl_preALU_active_out,
        output wire                                     indirect_R0_R1_access_flag,
        output logic  unsigned [DATA_WIDTH - 1 : 0]     bit_mask,
        output logic                                    reg_ctl_ALU_bit_mask_a,
        output logic                                    reg_ctl_bit_load,
        output logic                                    reg_ctl_inv_bit_load,
        output logic                                    reg_ctl_CY_update,
        
        output logic                                     reg_ctl_DPTR_load_data,
        output logic                                     reg_ctl_DPTR_inc,
        output logic                                     reg_ctl_DPTR_read,
        output logic                                     reg_ctl_DPTR_write,
        output logic  unsigned [SP_INC_BITS - 1 : 0]     reg_ctl_stack_move_amount,
        output logic                                     reg_ctl_stack_read,
        output logic                                     reg_ctl_stack_write,
                
        output wire                                      SP_write_lock,
        output wire                                      DPTR_write_lock,
        output logic                                     reg_ctl_interruptable
        
        
);
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        logic                                   running;
        logic                                   reg_ctl_ALU_active;
        wire  unsigned [DATA_WIDTH - 1 : 0]     bit_mask_tmp;
        logic unsigned [2 : 0]                  int_gen_flag_sr;        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // registers
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        assign bit_mask_tmp = ($size(bit_mask_tmp))'(1 << bit_mask_shift);
        
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
                
                bit_mask <= 0;
                
                reg_ctl_ALU_bit_mask_a <= 0;
                
                reg_ctl_bit_load       <= 0;
                reg_ctl_inv_bit_load   <= 0;
                
                reg_ctl_CY_update      <= 0;
                            
                reg_ctl_DPTR_load_data <= 0;
                reg_ctl_DPTR_inc       <= 0;
                reg_ctl_DPTR_read      <= 0;
                reg_ctl_DPTR_write     <= 0;
                reg_ctl_stack_move_amount <= 0;
                reg_ctl_stack_read        <= 0;
                reg_ctl_stack_write       <= 0;
                reg_ctl_interruptable     <= 0;
                
                reg_size_of_instruction   <= 0;
                int_gen_flag_sr           <= 0;
                
            end else begin
                int_gen_flag_sr <= {int_gen_flag_sr[$high(int_gen_flag_sr) - 1 : 0], int_gen_flag};
                
                if (ctl_ALU_active & running & (~(int_gen_flag | (|int_gen_flag_sr))) ) begin
                    reg_ctl_ALU_cmd         <= ctl_ALU_cmd ;
                    reg_ctl_indirect_write  <= ctl_indirect_write;
                    reg_ctl_PSW_flag_update <= ctl_PSW_flag_update;
                    reg_ctl_load_A          <= ctl_load_A;
                    reg_data_wr_address     <= data_wr_address;
                    
                    reg_ctl_xread_enable    <= ctl_xread_enable;
                    reg_ctl_xwrite_enable   <= ctl_xwrite_enable;
                
                    if (pipeline_stall | reg_jump_active) begin
                        reg_IR              <= INST_NOP;
                    end else begin
                        reg_IR              <= IR;
                    end
                    
                    reg_PC                  <= PC;
                    reg_size_of_instruction <= size_of_instruction;
                    
                    reg_data_wr_address     <= data_wr_address;
                    reg_data_rd_address     <= data_rd_address;
                    reg_ctl_ALU_active      <= 1;
                    
                    reg_ALU_adjust_a        <= ALU_adjust_a;
                    reg_ALU_adjust_b        <= ALU_adjust_b;
                    
                    reg_ctl_ALU_data_a      <= ctl_ALU_data_a;
                    reg_ctl_ALU_data_b      <= ctl_ALU_data_b;
                    
                    reg_ctl_ALU_adjust_a    <= ctl_ALU_adjust_a;
                    reg_ctl_ALU_adjust_b    <= ctl_ALU_adjust_b;
                    reg_ctl_write_enable    <= ctl_write_enable;
                           
                    if (ctl_bit_mask_invert) begin
                        bit_mask <= ~bit_mask_tmp;
                    end else begin
                        bit_mask <= bit_mask_tmp;
                    end
                    
                    reg_ctl_ALU_bit_mask_a <= ctl_ALU_bit_mask_a;
                    
                    reg_ctl_bit_load       <= ctl_bit_load;
                    reg_ctl_inv_bit_load   <= ctl_inv_bit_load;
                    
                    reg_ctl_CY_update      <= ctl_CY_update;
                    
                    if (pipeline_stall | reg_jump_active) begin
                        reg_ctl_DPTR_load_data    <= 0;
                        reg_ctl_DPTR_inc          <= 0;
                        reg_ctl_stack_move_amount <= 0;
                    end else begin
                        reg_ctl_DPTR_load_data    <= ctl_DPTR_load_data;
                        reg_ctl_DPTR_inc          <= ctl_DPTR_inc;
                        reg_ctl_stack_move_amount <= ctl_stack_move_amount;
                    end     
                    
                    reg_ctl_DPTR_read         <= ctl_DPTR_read;
                    reg_ctl_DPTR_write        <= ctl_DPTR_write;
                    reg_ctl_stack_read        <= ctl_stack_read;
                    reg_ctl_stack_write       <= ctl_stack_write;
                    reg_ctl_interruptable     <= ctl_interruptable;
                                                    
                end else begin
                    reg_ctl_ALU_active      <= 0;
                    if (int_gen_flag_sr[0] | int_gen_flag) begin
                        reg_ALU_adjust_b     <= 0;
                        reg_ctl_ALU_data_b   <= 0;
                        reg_ctl_ALU_adjust_b <= 1'b1;
                        reg_ctl_ALU_data_a   <= 1'b0;
                        reg_ctl_ALU_adjust_a <= 1'b0;
                        reg_ctl_stack_write  <= 1'b1;
                    end else begin
                        reg_ALU_adjust_b     <= ALU_adjust_b;
                        reg_ctl_ALU_data_b   <= ctl_ALU_data_b;
                        reg_ctl_ALU_adjust_b <= ctl_ALU_adjust_b;
                        reg_ctl_ALU_data_a   <= ctl_ALU_data_a;
                        reg_ctl_ALU_adjust_a <= ctl_ALU_adjust_a;
                        reg_ctl_stack_write  <= ctl_stack_write;
                    end
                    
                    if (int_gen_flag) begin
                        reg_ctl_ALU_cmd <= ($size(reg_ctl_ALU_cmd))'(1 << OP_ALU_SAVE_INT_PC_LOW);
                    end else if (int_gen_flag_sr [0]) begin
                        reg_ctl_ALU_cmd <= ($size(reg_ctl_ALU_cmd))'(1 << OP_ALU_SAVE_INT_PC_HIGH);
                    end else begin
                        reg_ctl_ALU_cmd <= ctl_ALU_cmd;
                    end
                    
                    reg_ctl_DPTR_load_data    <= 0;
                    reg_ctl_DPTR_inc          <= 0;
                    reg_ctl_stack_move_amount <= 0;
                        
                end
            end
        end : reg_proc
    
        assign ctl_preALU_active_out = reg_ctl_ALU_active;
        
        assign indirect_R0_R1_access_flag = ((!reg_data_wr_address [15 : 3]) 
                 && (reg_data_wr_address[2:1] == active_reg_bank_index) && reg_ctl_ALU_active) ? 1'b1 : 1'b0; 
            
        assign SP_write_lock = (((reg_data_wr_address == {8'd0, SPL_ADDR}) || (reg_data_wr_address == {8'd0, SPH_ADDR})) 
                   && reg_ctl_ALU_active && (~reg_ctl_indirect_write) && (~reg_ctl_stack_write) ) ? 1'b1 : 1'b0;
        assign DPTR_write_lock = (((reg_data_wr_address == {8'd0, DPL_ADDR}) || (reg_data_wr_address == {8'd0, DPH_ADDR}) )
                    && reg_ctl_ALU_active && (~reg_ctl_indirect_write) && (~reg_ctl_stack_write) ) ? 1'b1 : 1'b0;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_RUN, S_HOLD} states = S_RUN;
                
        localparam FSM_NUM_OF_STATES = states.num();
        logic [FSM_NUM_OF_STATES - 1:0] current_state = 0, next_state;
                
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
            
            ctl_RL_A = 0;
            ctl_RLC_A = 0;
            ctl_RR_A = 0;
            ctl_RRC_A = 0;
                    
            
            case (1'b1) // synthesis parallel_case 
                
                current_state[S_RUN]: begin
                    
                    if (ctl_pipeline_hold) begin
                        running = 0;
                        next_state [S_HOLD] = 1;
                    end else begin
                        next_state [S_RUN] = 1; 
                    end
                    
                    case (reg_IR[23 : 16]) // synthesis parallel_case 
                        
                        INST_RL_A : begin
                            ctl_RL_A = 1;
                        end
                        
                        INST_RLC_A : begin
                            ctl_RLC_A = 1;
                        end
                        
                        INST_RR_A : begin
                            ctl_RR_A = 1;
                        end
                        
                        INST_RRC_A : begin
                            ctl_RRC_A = 1;
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
            
endmodule : fast_core_pre_ALU

`default_nettype wire
