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
//    Arithmetic Logic Unit for FP51 fast core. 
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// fast_core_ALU
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//   
// See Also:
//    
//-----------------------------------------------------------------------------

module fast_core_ALU (
        
        //========== INPUT ==========
        
        input   wire                            clk,
        input   wire                            reset_n,
        
        input   wire                            int_gen,        
        input   wire  unsigned [7 : 0]          int_addr,
        
        
        input   wire unsigned [ADDR_WIDTH - 1 : 0]      DPTR,
        input   wire unsigned [ADDR_WIDTH - 1 : 0]      SP,
        
        input   wire unsigned [7 : 0]           R0,
        input   wire unsigned [7 : 0]           R1,
        input   wire unsigned [7 : 0]           XPAGE,
        
        input   wire unsigned [1 : 0]           active_reg_bank_index,
        
        input   wire unsigned [23 : 0]              IR,
        input   wire unsigned [PC_BITWIDTH - 1 : 0] PC,
        input   wire  unsigned [1 : 0]              size_of_instruction,
        
        input   wire                                ctl_interruptable,
        input   wire                                ctl_DPTR_load_data,
        input   wire                                ctl_DPTR_inc,
        input   wire                                ctl_DPTR_read,
        input   wire                                ctl_DPTR_write,
                
        input   wire  unsigned [SP_INC_BITS - 1 : 0]    ctl_stack_move_amount,
        input   wire                                    ctl_stack_read,
        input   wire                                    ctl_stack_write,

        input   wire unsigned [7 : 0]           bit_mask,
        input   wire                            ctl_ALU_bit_mask_a,
        
        input   wire                            ctl_bit_load,
        input   wire                            ctl_inv_bit_load,
        
        input   wire                            ctl_CY_update,
        
        input   wire                            CY_current,
        input   wire                            AC_current,
        input   wire unsigned [7 : 0]           ACC,
        input   wire unsigned [7 : 0]           PSW,
        input   wire unsigned [7 : 0]           B,
        input   wire unsigned [7 : 0]           ALU_adjust_a,
        input   wire unsigned [7 : 0]           ALU_adjust_b,
        
        input   wire                            ctl_RL_A,
        input   wire                            ctl_RLC_A,
        input   wire                            ctl_RR_A,
        input   wire                            ctl_RRC_A,
        
        input   wire                            ctl_xread_enable,
        input   wire                            ctl_xwrite_enable,
      
        input   wire [ALU_NUM_OF_OPERATIONS - 1 : 0]    ctl_ALU_cmd,
        input   wire unsigned [7 : 0]           onchip_data_in,
        
        input   wire                            indirect_R0_R1_access_flag,
        
        input   wire                            SP_write_lock,
        input   wire                            DPTR_write_lock,
        
        input   wire                            ctl_write_enable,
        input   wire                            ctl_indirect_write,
        input   wire  unsigned [15 : 0]         data_rd_address,
        input   wire  unsigned [15 : 0]         data_wr_address,
        
        input   wire                            ctl_ALU_active,
        input   wire                            ctl_ALU_adjust_a,
        input   wire                            ctl_ALU_adjust_b,
        input   wire                            ctl_ALU_data_a,
        input   wire                            ctl_ALU_data_b,
        input   wire                            ctl_PSW_flag_update,
        input   wire                            ctl_load_A,
        
        input   wire  unsigned [7 : 0]          SFR_MOVC_data_in,
        input   wire                            SFR_ctl_load_MOVC_data,
        
        //========== OUTPUT ==========
        
        output  logic  unsigned [7 : 0]         ALU_op_a,
        output  logic  unsigned [7 : 0]         ALU_op_b,   
        
        output logic                            reg_indirect_R0_R1_access_flag,
        output logic   unsigned [15 : 0]        reg_data_wr_address,
        
        output logic                            reg_ctl_indirect_write,
        output logic                            reg_ctl_PSW_flag_update,
        output logic                            reg_ctl_load_A,
        output logic                            reg_ctl_RL_A,
        output logic                            reg_ctl_RLC_A,
        output logic                            reg_ctl_RR_A,
        output logic                            reg_ctl_RRC_A,
        output logic unsigned [23 : 0]          reg_IR,
        output logic unsigned [PC_BITWIDTH - 1 : 0] reg_PC,
        output logic  unsigned [1 : 0]          reg_size_of_instruction,
                
        output logic                            reg_onchip_data_we,
        output wire   unsigned [7 : 0]          onchip_data_to_write,
        output logic                            ctl_pipeline_hold,
        output logic                            ctl_pipeline_resume,
        
        output wire                             CY,
        output wire                             AC,
        output wire                             P,
        output wire                             OV,
        output logic unsigned [15 : 0]          MOVC_addr,
        output logic                            ctl_load_MOVC,
        output logic                            reg_ctl_CY_update,
    
        output logic                                     reg_ctl_DPTR_load_data,
        output logic                                     reg_ctl_DPTR_inc,
        output logic                                     reg_ctl_DPTR_read,
        output logic                                     reg_ctl_DPTR_write,
        output logic  unsigned [SP_INC_BITS - 1 : 0]     reg_ctl_stack_move_amount,
        output logic                                     reg_ctl_stack_read,
        output logic                                     reg_ctl_stack_write,
        
        output logic                                     reg_jump_active,
        
        output logic                                     reg_SP_write_lock,
        output logic                                     reg_DPTR_write_lock,
        output logic unsigned [PC_BITWIDTH - 1 : 0]      next_PC,
        output logic unsigned [DATA_WIDTH - 1 : 0]       stack_read_data,
        output wire                                      int_gen_flag,
        output logic unsigned [7 : 0]                    int_addr_reg,
        output logic unsigned [PC_BITWIDTH - 1 : 0]      INT_PC,
        output logic                                     reg_ret_int,
        
        //===================================================================
        // Debug
        //===================================================================
        
        input wire                                      debug_data_write,
        input wire unsigned [PC_BITWIDTH - 1 : 0]       debug_data_write_addr,
        input wire unsigned                             debug_wr_indirect1_direct0,
        input wire unsigned [DATA_WIDTH - 1 : 0]        debug_data_write_data
);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        logic  [ALU_NUM_OF_OPERATIONS - 1 : 0]  reg_ctl_ALU_cmd;
        logic                                   ctl_ALU_active_d1;
        logic  unsigned [8 : 0]                 ALU_out_i;
        
        wire   unsigned [7 : 0]                 actual_ALU_op_a, actual_ALU_op_b;
                
        logic  unsigned [7 : 0]                 reg_onchip_data_to_write;
        
        wire                                    aux_bit;
        wire                                    sign_ext;
        
        logic  unsigned [15 : 0]                A_times_B_or_BCD;
        
        logic                                   OV_ext;
        logic                                   CY_ext;
        wire                                    div_ov_flag;
        wire  unsigned [7 : 0]                  div_quotient;
        wire  unsigned [7 : 0]                  div_remainder;
        wire                                    DA_c_flag;
        wire  unsigned [7 : 0]                  DA_sum;
              
     
        logic                                   reg_use_ALU_op_a;
        logic                                   bit_op;
        
        logic                                   reg_ctl_bit_load, reg_ctl_inv_bit_load;
                
        logic unsigned [7 : 0]                  reg_bit_mask;
        logic unsigned [7 : 0]                  reg_C_bit_mask;
        
        wire unsigned [PC_BITWIDTH - 1 : 0]     PC_plus_2, PC_plus_3;
        
        logic [OP_NEXT_PC_NUM_OF_OPERATIONS - 1 : 0]    ctl_cmd_next_PC;
        
        logic                                   ctl_jump_active;
        logic                                   ctl_ret_jump_active, ctl_ret_jump_active_d1;
                
        logic                                   int_gen_reg;
        logic unsigned [6 : 0]                  int_gen_flag_sr;
        logic unsigned [7 : 0]                  reg_jump_active_sr;
        logic                                   ctl_ret_int;
        logic                                   ctl_interruptable_d1;
        logic                                   debug_data_write_d1;
        logic                                   ctl_write_allowed;
        
        wire  unsigned [ADDR_WIDTH - 1 : 0]     DPTR_plus_1;
        wire  unsigned [ADDR_WIDTH - 1 : 0]     SP_adjust;
                
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // registers
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        assign int_gen_flag = int_gen_reg & ctl_interruptable_d1; 
            
        always_ff @(posedge clk, negedge reset_n) begin : reg_proc
            if (!reset_n) begin
                ctl_ALU_active_d1 <= 0;
                reg_indirect_R0_R1_access_flag <= 0;
                reg_ctl_ALU_cmd         <= 0;
                reg_ctl_indirect_write  <= 0;
                
                reg_ctl_PSW_flag_update <= 0;
                reg_ctl_load_A          <= 0;
                reg_data_wr_address     <= 0;
                reg_IR                  <= 0;
                reg_PC                  <= 0;
                
                reg_ctl_RL_A            <= 0;
                reg_ctl_RLC_A           <= 0;
                reg_ctl_RR_A            <= 0;
                reg_ctl_RRC_A           <= 0;
                  
                reg_onchip_data_to_write <= 0;
                
                reg_onchip_data_we       <= 0;
               
                reg_use_ALU_op_a <= 0;
                
                reg_bit_mask  <= 0;
                
                reg_ctl_bit_load         <= 0;
                reg_ctl_inv_bit_load     <= 0;
                                
                reg_ctl_CY_update        <= 0;
                
                reg_SP_write_lock        <= 0;
                reg_DPTR_write_lock      <= 0;
                
                reg_ctl_DPTR_load_data    <= 0;
                reg_ctl_DPTR_inc          <= 0;
                reg_ctl_DPTR_read         <= 0;
                reg_ctl_DPTR_write        <= 0;
                reg_ctl_stack_move_amount <= 0;
                reg_ctl_stack_read        <= 0;
                reg_ctl_stack_write       <= 0;             
                stack_read_data           <= 0;
                
                int_gen_reg               <= 0;
                int_addr_reg              <= 0;
                                        
                INT_PC                    <= 0;
                
                reg_size_of_instruction   <= 0;
                int_gen_flag_sr           <= 0;
                
                reg_jump_active_sr        <= 0;
                
                ctl_interruptable_d1      <= 0;
                
                debug_data_write_d1       <= 0;
                
            end else begin
                debug_data_write_d1       <= debug_data_write;
                
                ctl_ALU_active_d1         <= ctl_ALU_active;
                ctl_interruptable_d1      <= ctl_interruptable;
                
                int_gen_reg <= int_gen  | ((~ctl_interruptable_d1)& int_gen_reg);
                
                int_gen_flag_sr <= {int_gen_flag_sr[$high(int_gen_flag_sr) - 1 : 0], int_gen_flag};
                
                reg_jump_active_sr <= {reg_jump_active_sr [$high(reg_jump_active_sr) - 1 : 0], (ctl_jump_active | ctl_ret_jump_active_d1) & (~int_gen_flag)}; 
                    
                if (int_gen) begin
                    int_addr_reg  <= int_addr;
                end
                
                if (debug_data_write) begin
                    reg_data_wr_address      <= debug_data_write_addr;
                    reg_onchip_data_we       <= 1'b1;
                    reg_ctl_indirect_write   <= debug_wr_indirect1_direct0;
                    reg_onchip_data_to_write <= debug_data_write_data;
                                            
                end else if ((ctl_ALU_active) && (IR [23 : 16] != INST_NOP) && (!reg_jump_active) 
                     && ((!int_gen_reg) || (!ctl_interruptable_d1)) ) begin
                    reg_indirect_R0_R1_access_flag <= indirect_R0_R1_access_flag;
                    reg_ctl_ALU_cmd         <= ctl_ALU_cmd;
                    reg_ctl_indirect_write  <= ctl_indirect_write;
                    
                    reg_ctl_PSW_flag_update <= ctl_PSW_flag_update;
                    reg_ctl_load_A          <= ctl_load_A;
                    
                    if (ctl_stack_write) begin
                        reg_data_wr_address     <= SP + (ADDR_WIDTH)'(1) + 
                            {{(ADDR_WIDTH - SP_INC_BITS){reg_ctl_stack_move_amount[$high(reg_ctl_stack_move_amount)]}}, reg_ctl_stack_move_amount};

                    end else if (ctl_DPTR_write) begin
                        if (reg_ctl_DPTR_load_data) begin
                            reg_data_wr_address  <= reg_IR [15 : 0] + ($size(reg_data_wr_address))'(XRAM_ADDR_OFFSET_IN_BYTES);
                        end else begin
                            reg_data_wr_address  <= DPTR + ($size(reg_data_wr_address))'(XRAM_ADDR_OFFSET_IN_BYTES) + {15'd0, reg_ctl_DPTR_inc};
                        end
                    end else if (ctl_indirect_write) begin
                        if ((reg_data_wr_address == {13'd0, active_reg_bank_index, IR[16]} ) && (reg_onchip_data_we)) begin
                            reg_data_wr_address  <= {XPAGE & {8{ctl_xwrite_enable}}, onchip_data_to_write} + {7'd0, ctl_xwrite_enable, 8'd0};
                        end else if (IR [16]) begin
                            reg_data_wr_address <= {XPAGE & {8{ctl_xwrite_enable}}, R1} + {7'd0, ctl_xwrite_enable, 8'd0};
                        end else begin
                            reg_data_wr_address <= {XPAGE & {8{ctl_xwrite_enable}}, R0} + {7'd0, ctl_xwrite_enable, 8'd0};
                        end
                        
                    end else begin
                        reg_data_wr_address     <= data_wr_address;
                    end
                   
                    reg_IR   <= IR;
                    reg_PC   <= PC;
                    
                    reg_size_of_instruction <= size_of_instruction;
                    
                    reg_ctl_RL_A            <= ctl_RL_A;
                    reg_ctl_RLC_A           <= ctl_RLC_A;
                    reg_ctl_RR_A            <= ctl_RR_A;
                    reg_ctl_RRC_A           <= ctl_RRC_A;
                                        
                    reg_onchip_data_we <= (ctl_write_enable | ctl_RL_A | ctl_RLC_A | ctl_RR_A | ctl_RRC_A) & ctl_write_allowed;
                                                
                    
                    reg_use_ALU_op_a        <= ctl_ALU_adjust_a | ctl_ALU_data_a | ctl_ALU_bit_mask_a |
                         ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_2_LOW] | ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_2_HIGH] |
                         ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_3_LOW] | ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_3_HIGH];
                
                    reg_bit_mask            <= bit_mask;
                    
                    reg_ctl_bit_load        <= ctl_bit_load;
                    reg_ctl_inv_bit_load    <= ctl_inv_bit_load;
                    
                    reg_ctl_CY_update       <= ctl_CY_update;
                    
                    reg_ctl_DPTR_load_data    <= ctl_DPTR_load_data;
                    reg_ctl_DPTR_inc          <= ctl_DPTR_inc;
                    reg_ctl_DPTR_read         <= ctl_DPTR_read;
                    reg_ctl_DPTR_write        <= ctl_DPTR_write;
                    reg_ctl_stack_move_amount <= ctl_stack_move_amount;
                    reg_ctl_stack_read        <= ctl_stack_read;
                    reg_ctl_stack_write       <= ctl_stack_write;               
                        
                    reg_SP_write_lock        <= SP_write_lock;
                    reg_DPTR_write_lock      <= DPTR_write_lock;
                    
                    if (ctl_stack_read) begin
                        stack_read_data <= onchip_data_in;
                    end
                                    
                end else begin
                    reg_ctl_RL_A       <= 0;
                    reg_ctl_RLC_A      <= 0;
                    reg_ctl_RR_A       <= 0;
                    reg_ctl_RRC_A      <= 0;
                    reg_ctl_load_A     <= 0;
                    reg_ctl_PSW_flag_update <= 0;
                    reg_ctl_indirect_write  <= 0;
                    
                    reg_IR                  <= INST_NOP;
                    reg_indirect_R0_R1_access_flag <= 0;
                               
                    reg_bit_mask            <= 0;
                    
                    reg_ctl_bit_load        <= 0;
                    reg_ctl_inv_bit_load    <= 0;
                    
                    reg_ctl_CY_update       <= 0;
                    
                    reg_ctl_DPTR_load_data    <= 0;
                    reg_ctl_DPTR_inc          <= 0;
                    reg_ctl_DPTR_read         <= 0;
                    reg_ctl_DPTR_write        <= 0;
                    
                    reg_ctl_stack_read        <= 0;
                                                            
                    reg_SP_write_lock         <= 0;
                    reg_DPTR_write_lock       <= 0;
                    stack_read_data           <= 0;
                
                    if (int_gen_flag_sr [$high(int_gen_flag_sr) : 1]) begin
                        INT_PC <= {8'd0, int_addr_reg};
                    end else if (reg_jump_active_sr) begin
                        INT_PC <= next_PC;
                    end else if (ctl_ALU_active | ctl_ALU_active_d1) begin
                        INT_PC <= PC;
                    end else begin
                        INT_PC <= reg_PC + reg_size_of_instruction;
                    end
                    
                    
                    if (int_gen_flag_sr[0] | int_gen_flag_sr[1]) begin
                        reg_data_wr_address       <= SP + ($size(reg_data_wr_address))'(1);
                        reg_onchip_data_we        <= 1'b1;
                        reg_use_ALU_op_a          <= 1'b1;
                        reg_ctl_ALU_cmd           <= ctl_ALU_cmd;
                        reg_ctl_stack_write       <= 1'b1;
                    end else begin 
                        reg_onchip_data_we        <= 1'b0;
                        reg_use_ALU_op_a          <= 0;
                        reg_ctl_ALU_cmd           <= 0;
                        reg_ctl_stack_write       <= 0;
                        
                    end
                    
                    if (int_gen_flag_sr[0] | int_gen_flag) begin
                        reg_ctl_stack_move_amount <= 3'b001;
                    end else begin
                        reg_ctl_stack_move_amount <= 0;
                    end
                end
                
            end
            
        end : reg_proc
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // jump active
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin : reg_jump_active_proc
            if (!reset_n) begin
                reg_jump_active <= 0;
                ctl_ret_jump_active_d1 <= 0;
            end else begin
                ctl_ret_jump_active_d1 <= ctl_ret_jump_active;
                reg_jump_active <= (ctl_jump_active | ctl_ret_jump_active_d1) & (~int_gen_flag) & (~(|int_gen_flag_sr));
            end
        end : reg_jump_active_proc
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // interrupt return
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        always_ff @(posedge clk, negedge reset_n) begin : reg_ret_int_proc
            if (!reset_n) begin
                reg_ret_int <= 0;
            end else begin
                reg_ret_int <= ctl_ret_int;
            end
        end : reg_ret_int_proc
        
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ALU input
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        assign PC_plus_2 = PC + ($size(PC_plus_2))'(2);
        assign PC_plus_3 = PC + ($size(PC_plus_3))'(3);
        
        // ALU input a
        always_ff @(posedge clk) begin : ALU_op_a_proc
            case (1'b1)
                ctl_ALU_adjust_a : begin
                    ALU_op_a <= ALU_adjust_a; 
                end
                
                ctl_ALU_data_a : begin
                    ALU_op_a <= IR [7 : 0];
                end
                
                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_2_LOW] : begin
                    ALU_op_a <= PC_plus_2 [7 : 0];
                end
                    
                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_2_HIGH] : begin
                    ALU_op_a <= PC_plus_2 [15 : 8];
                end
                
                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_3_LOW] : begin
                    ALU_op_a <= PC_plus_3 [7 : 0];
                end
                    
                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_3_HIGH] : begin
                    ALU_op_a <= PC_plus_3 [15 : 8];
                end
                
                ctl_ALU_cmd [OP_ALU_SAVE_INT_PC_HIGH] : begin
                    ALU_op_a <= INT_PC [15 : 8];
                end
                
                ctl_ALU_cmd [OP_ALU_SAVE_INT_PC_LOW] : begin
                    ALU_op_a <= INT_PC [7 : 0];
                end
                
                default : begin
                    ALU_op_a <= bit_mask;
                end
            endcase
        end : ALU_op_a_proc
        
        assign actual_ALU_op_a = (reg_use_ALU_op_a) ? ALU_op_a : ACC;
        
        assign DPTR_plus_1 = DPTR + ($size(DPTR_plus_1))'(1);
        assign SP_adjust = SP + 
            {{(ADDR_WIDTH - SP_INC_BITS){reg_ctl_stack_move_amount[$high(reg_ctl_stack_move_amount)]}}, reg_ctl_stack_move_amount};
                            
                                
        // ALU input b
        always_ff @(posedge clk) begin : ALU_op_b_proc
                        
            case (1'b1) // synthesis parallel_case 
                
                ctl_ALU_adjust_b : begin
                    ALU_op_b <= ALU_adjust_b; 
                end
                
                ctl_ALU_data_b : begin
                    ALU_op_b <= IR [15 : 8];    
                end
                
                reg_ctl_ALU_cmd [OP_ALU_MUL_2ND] : begin
                    ALU_op_b <= A_times_B_or_BCD [7 : 0];
                end
                
                reg_ctl_ALU_cmd [OP_ALU_DIV_SAVE_A] : begin
                    ALU_op_b <= div_quotient;
                end
                    
                reg_ctl_ALU_cmd [OP_ALU_DIV_SAVE_B] : begin
                    ALU_op_b <= div_remainder;
                end
                
                default : begin
                    
                    
                    //ALU_op_b <= onchip_data_in;
                    
                    if ( (data_rd_address == reg_data_wr_address) && (reg_onchip_data_we) ) begin
                        ALU_op_b <= onchip_data_to_write;
                    end else if (!ctl_stack_read) begin
                        case (data_rd_address) // synthesis parallel_case 
                            ACC_ADDR : begin
                                if (reg_ctl_load_A) begin
                                    ALU_op_b <= ALU_op_b;
                                end else if (SFR_ctl_load_MOVC_data) begin
                                    ALU_op_b <= SFR_MOVC_data_in;
                                end else begin
                                    ALU_op_b <= onchip_data_in; 
                                end     
                            end
                            
                            
                            PSW_ADDR : begin
                                case (1'b1) // synthesis parallel_case 
                                    reg_ctl_PSW_flag_update : begin
                                        ALU_op_b <= {CY, AC, PSW[5 : 3], OV, PSW[1], P};
                                    end
                        
                                    reg_ctl_RLC_A : begin
                                        ALU_op_b <= {ACC[7], PSW [6 : 0]};
                                    end
                                    
                                    reg_ctl_RRC_A : begin
                                        ALU_op_b <= {ACC[0], PSW [6 : 0]};
                                    end
                                    
                                    reg_ctl_CY_update : begin
                                        ALU_op_b <= {CY, PSW [6 : 0]};
                                    end
                                    
                                    default : begin
                                        ALU_op_b <= onchip_data_in;
                                    end
                                endcase     
                            end
                        
                            DPL_ADDR : begin
                                if (reg_ctl_DPTR_load_data) begin
                                    ALU_op_b <= reg_IR [7 : 0];
                                end else if (reg_ctl_DPTR_inc) begin
                                    ALU_op_b <= DPTR_plus_1 [7 : 0];
                                end else begin
                                    ALU_op_b <= DPTR [7 : 0];           
                                end     
                            end
                            
                            DPH_ADDR : begin
                                if (reg_ctl_DPTR_load_data) begin
                                    ALU_op_b <= reg_IR [15 : 8];
                                end else if (reg_ctl_DPTR_inc) begin
                                    ALU_op_b <= DPTR_plus_1 [15 : 8];
                                end else begin
                                    ALU_op_b <= DPTR [15 : 8];          
                                end     
                            end
                        
                            SPL_ADDR : begin
                                ALU_op_b <= SP_adjust [7 : 0];      
                            end
                            
                        /*  SPH_ADDR : begin
                                ALU_op_b <= SP_adjust [15 : 8];     
                            end
                        */  
                            default : begin
                                ALU_op_b <= onchip_data_in;
                            end
                            
                        endcase
                    end else begin
                        ALU_op_b <= onchip_data_in;
                    end

                    
                end
            endcase
            
        end : ALU_op_b_proc
        
        
        // for MOV bit, C
        always_ff @(posedge clk) begin : reg_C_bit_mask_proc
            if ((reg_data_wr_address == PSW_ADDR) && (reg_onchip_data_we)) begin
                reg_C_bit_mask <= bit_mask & {8{onchip_data_to_write[PSW_CY_INDEX]}};
            end else begin
                case (1'b1) // synthesis parallel_case 
                    reg_ctl_PSW_flag_update | reg_ctl_CY_update : begin
                        reg_C_bit_mask <= bit_mask & {8{CY}};
                    end
        
                    reg_ctl_RLC_A : begin
                        reg_C_bit_mask <= bit_mask & {8{ACC[7]}};
                    end
                    
                    reg_ctl_RRC_A : begin
                        reg_C_bit_mask <= bit_mask & {8{ACC[0]}};
                    end
                    
                    default : begin
                        reg_C_bit_mask <= bit_mask & {8{CY_current}};
                    end
                    
                endcase
            end
            
        end : reg_C_bit_mask_proc
        
        always_comb begin
            if (reg_ctl_bit_load) begin
                bit_op = |(ALU_op_b & reg_bit_mask);
            end else if (reg_ctl_inv_bit_load) begin
                bit_op = ~(|(ALU_op_b & reg_bit_mask));
            end else begin
                bit_op = CY_current;
            end
        end
            
        assign actual_ALU_op_b = ALU_op_b;
            
        
        // op_ALU cast for debug, one-hot translation, 
        // enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "op_ALU" variable
        
        // synthesis translate_off
        ///////////////////////////////////////////////////////////////////////
            op_ALU_t op_ALU_debug;
            always_comb begin : op_ALU_cast_for_debug
                for (int i = 0; i < ALU_NUM_OF_OPERATIONS; ++i) begin
                    if (reg_ctl_ALU_cmd[i]) begin
                        $cast(op_ALU_debug, i);
                    end
                end
            end : op_ALU_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on   
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ALU core
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        assign onchip_data_to_write = debug_data_write_d1 ? reg_onchip_data_to_write : ALU_out_i [7 : 0];
        
        assign CY  = reg_ctl_ALU_cmd[OP_ALU_DA_WR] ? CY_ext : ALU_out_i [8];
        assign AC  = aux_bit ^ ALU_out_i[4];
        assign P   = ^(ALU_out_i [7 : 0]);
        assign OV  = (reg_ctl_ALU_cmd[OP_ALU_MUL_3RD] | reg_ctl_ALU_cmd[OP_ALU_DIV_SAVE_B]) 
                ? OV_ext : (sign_ext ^ CY ^ ALU_out_i[7]);
        
        assign aux_bit = actual_ALU_op_a [4] ^ actual_ALU_op_b[4];
        assign sign_ext = actual_ALU_op_a [7] ^ actual_ALU_op_b[7];
        
        always_ff @(posedge clk, negedge reset_n) begin : flag_proc
            if (!reset_n) begin
                OV_ext <= 0;
            end else if (reg_ctl_ALU_cmd [OP_ALU_MUL_2ND]) begin
                OV_ext <= |(A_times_B_or_BCD [15 : 8]);
            end else if (reg_ctl_ALU_cmd [OP_ALU_DIV_SAVE_A]) begin
                OV_ext <= div_ov_flag;              
            end
        end : flag_proc
                        
        always_comb begin : ALU_proc
            case (1'b1) // synthesis parallel_case 
                
                reg_ctl_ALU_cmd [OP_ALU_ADD] : begin
                    ALU_out_i = {1'b0, actual_ALU_op_a} + {1'b0, actual_ALU_op_b};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_ADDC] : begin
                    ALU_out_i = {1'b0, actual_ALU_op_a} + {1'b0, actual_ALU_op_b} + {8'h0, CY_current};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_A_DIRECT_PASS] : begin
                    ALU_out_i = {1'b0, actual_ALU_op_a};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_SUB] : begin
                    ALU_out_i = {1'b0, actual_ALU_op_a} - {1'b0, actual_ALU_op_b};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_SUB_NEG] : begin
                    ALU_out_i = {1'b0, actual_ALU_op_b} - {1'b0, actual_ALU_op_a} ;
                end
                
                reg_ctl_ALU_cmd [OP_ALU_SUBC] : begin
                    ALU_out_i = {1'b0, actual_ALU_op_a} - {1'b0, actual_ALU_op_b} - {8'h0, CY_current};
                end
                                                
                reg_ctl_ALU_cmd [OP_ALU_AND] : begin
                    ALU_out_i = {CY_current, actual_ALU_op_a} & {bit_op, actual_ALU_op_b};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_OR] : begin
                    ALU_out_i = {CY_current, actual_ALU_op_a} | {bit_op, actual_ALU_op_b};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_MUL_2ND] : begin
                    ALU_out_i = {1'b0, A_times_B_or_BCD [15 : 8]};
                end
                
                reg_ctl_ALU_cmd [OP_ALU_MOV_BIT_C] : begin
                    ALU_out_i = {1'b0, (actual_ALU_op_b & (~reg_bit_mask)) | reg_C_bit_mask};
                end
                
                reg_ctl_RL_A : begin
                    ALU_out_i = {1'b0, ACC [6 : 0], ACC[7]};
                end
                
                reg_ctl_RLC_A : begin
                    ALU_out_i = {ACC, CY_current};
                end
                
                reg_ctl_RR_A : begin
                    ALU_out_i = {1'b0, ACC[0], ACC [7 : 1]};
                end
                
                reg_ctl_RRC_A : begin
                    ALU_out_i = {1'b0, CY_current, ACC [7 : 1]};
                end
                                                
                default : begin
                    ALU_out_i = {1'b0, actual_ALU_op_a} ^ {bit_op, actual_ALU_op_b};
                end
                
            endcase
            
        
        end : ALU_proc  
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                A_times_B_or_BCD <= 0;
            end else if (reg_ctl_ALU_cmd [OP_ALU_MUL]) begin
                A_times_B_or_BCD <= ACC * B;
            end else if (reg_ctl_ALU_cmd[OP_ALU_DA_SAVE]) begin
                A_times_B_or_BCD <= {8'd0, DA_sum};
            end else if (reg_ctl_ALU_cmd[OP_ALU_SWAP_A]) begin
                A_times_B_or_BCD <= {ACC [3 : 0], ACC [7 : 4], 8'd0};
            end else if (reg_ctl_ALU_cmd[OP_ALU_XCHD]) begin
                A_times_B_or_BCD <= {ALU_op_b[7 : 4], ACC [3 : 0], ACC [7 : 4], ALU_op_b [3 : 0]};
            end
        end
        
        
        SRT_Radix4_division #(.DATA_WIDTH (8)) div_i (.*,
                .enable_in (reg_ctl_ALU_cmd[OP_ALU_DIV_START]),
                .dividend (ACC),
                .divisor (B),
                .ov_flag (div_ov_flag),
                .quotient (div_quotient),
                .remainder (div_remainder));
            
        DA_A DA_A_i(.*,
            .enable_in (reg_ctl_ALU_cmd [OP_ALU_DA_START]),
            .CY (CY_current),
            .AC (AC_current),
            .ACC (ACC),
            .c_flag (DA_c_flag),
            .sum (DA_sum));
        
        always_ff @(posedge clk, negedge reset_n) begin 
            if (!reset_n) begin
                CY_ext <= 0;
            end else if (reg_ctl_ALU_cmd[OP_ALU_DA_SAVE]) begin
                CY_ext <= DA_c_flag;
            end
        end
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // DPTR
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                MOVC_addr <= 0;
            end else if (reg_ctl_ALU_cmd[OP_ALU_MOVC_A_DPTR]) begin
                MOVC_addr <= {8'd0, ACC} + DPTR;
            end else if (reg_ctl_ALU_cmd[OP_ALU_MOVC_A_PC]) begin
                MOVC_addr <= {8'd0, ACC} + reg_PC;
            end
        end
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                ctl_load_MOVC <= 0;
            end else begin
                ctl_load_MOVC <= reg_ctl_ALU_cmd[OP_ALU_MOVC_A_DPTR] | reg_ctl_ALU_cmd[OP_ALU_MOVC_A_PC];
            end
        end
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // next PC
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
        always_ff @(posedge clk, negedge reset_n) begin : next_PC_proc
            if (!reset_n) begin
                next_PC <= 0;
            end else begin
                
                case (1'b1) // synthesis parallel_case 
                                                
                    ctl_cmd_next_PC[OP_ADDR_JMP_REL_ADDR_2_BYTE] : begin
                        // two byte instructions, one byte relative address
                        next_PC <= reg_PC + {{8{reg_IR[15]}}, reg_IR [15 : 8]} + ($size(next_PC))'(2);
                    end
                            
                    ctl_cmd_next_PC[OP_ADDR_JMP_REL_ADDR_3_BYTE] : begin
                        // three byte instructions, one byte relative address
                        next_PC <= reg_PC + {{8{reg_IR[7]}}, reg_IR [7 : 0]} + ($size(next_PC))'(3);
                    end
                                
                    ctl_cmd_next_PC[OP_ADDR_ACALL_AJMP] : begin
                        // two byte instructions
                        next_PC <= {PC_plus_2 [15 : 11], reg_IR [23 : 21], reg_IR [15 : 8]};
                    end
                    
                    ctl_cmd_next_PC [OP_ADDR_LCALL_LJMP] : begin
                        next_PC <= reg_IR [15 : 0];
                    end
                    
                    ctl_cmd_next_PC [OP_ADDR_JMP_A_DPTR] : begin
                        next_PC <= ACC + DPTR;
                    end
                            
                    ctl_cmd_next_PC [OP_ADDR_PC_RESTORE_FROM_STACK] : begin
                        next_PC <= {next_PC [7 : 0], ALU_op_b}; 
                    end
                    
                    ctl_cmd_next_PC [OP_ADDR_NOP] : begin
                        
                    end
                                        
                    default : begin
                        if (!reg_jump_active_sr) begin
                            next_PC <= reg_PC;
                        end
                    end
                
                endcase
            end
        end : next_PC_proc
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_RUN, S_NOP_RUN, S_RET, S_RUN_EXT} states = 0;
                
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
            
            ctl_pipeline_hold = 0;
            ctl_pipeline_resume = 0;
            
            ctl_jump_active = 0;
            
            ctl_cmd_next_PC = 0;
            
            ctl_ret_jump_active = 0;
            
            ctl_ret_int = 0;
            
            ctl_write_allowed = 1'b1;
            
            case (1'b1) // synthesis parallel_case 
                
                current_state[S_RUN]: begin
                                    
                    if (ctl_ALU_active_d1) begin
                        casex (reg_IR[23 : 16]) // synthesis parallel_case 
                            
                            INST_JC : begin
                                ctl_jump_active = CY_current;
                                next_state[S_NOP_RUN] = 1'b1;   
                                
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_2_BYTE] = 1;
                            end
                            
                            INST_JNC : begin
                                ctl_jump_active = ~CY_current;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_2_BYTE] = 1;
                            end
                            
                            INST_JB : begin
                                ctl_jump_active = bit_op;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            INST_JNB : begin
                                ctl_jump_active = bit_op;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            INST_JBC : begin
                                ctl_jump_active = bit_op;
                                ctl_write_allowed = bit_op;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            INST_JZ : begin
                                if (!ACC) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_2_BYTE] = 1;
                            end
                            
                            INST_JNZ : begin
                                if (ACC) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_2_BYTE] = 1;
                            end
                            
                            INST_CJNE_A_DIR_REL : begin
                                if (ACC != ALU_op_b) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            INST_CJNE_A_DATA_REL : begin
                                if (ACC != reg_IR [15 : 8]) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            {INST_CJNE_R_DATA_REL_MSB, 3'b???} : begin
                                if (ALU_op_b != reg_IR [15 : 8]) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            {INST_CJNE_AT_R_DATA_REL_MSB, 1'b?} : begin
                                if (ALU_op_b != reg_IR [15 : 8]) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            {INST_DJNZ_R_REL_MSB, 3'b???} : begin
                                if (ALU_op_b != 8'd1) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_2_BYTE] = 1;
                            end
                            
                            INST_DJNZ_DIR_REL : begin
                                if (onchip_data_to_write) begin
                                    ctl_jump_active = 1'b1;
                                end
                                
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_3_BYTE] = 1;
                            end
                            
                            INST_SJMP : begin
                                ctl_jump_active = 1'b1;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_JMP_REL_ADDR_2_BYTE] = 1;
                            end
                            
                            {3'b???, INST_AJMP_LSB} : begin
                                ctl_jump_active = 1'b1;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_ACALL_AJMP] = 1;
                            end
                            
                            {3'b???, INST_ACALL_LSB} : begin
                                ctl_jump_active = 1'b1;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_ACALL_AJMP] = 1;
                            end
                            
                            INST_LCALL : begin
                                ctl_jump_active = 1'b1;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_LCALL_LJMP] = 1;
                            end
                            
                            INST_LJMP : begin
                                ctl_jump_active = 1'b1;
                                next_state[S_NOP_RUN] = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_LCALL_LJMP] = 1;
                            end
                                                        
                            INST_JMP_A_DPTR : begin
                                ctl_jump_active = 1'b1;
                                next_state[S_NOP_RUN] = 1'b1;
                                
                                ctl_cmd_next_PC [OP_ADDR_JMP_A_DPTR] = 1;
                            end
                            
                            INST_RET : begin
                                ctl_ret_jump_active = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_PC_RESTORE_FROM_STACK] = 1;
                                next_state[S_RET] = 1'b1;
                            end
                            
                            INST_RETI : begin
                                ctl_ret_int = 1'b1;
                                ctl_ret_jump_active = 1'b1;
                                ctl_cmd_next_PC [OP_ADDR_PC_RESTORE_FROM_STACK] = 1;
                                next_state[S_RET] = 1'b1;
                            end
                                                                                    
                            default : begin
                                next_state [S_RUN] = 1;
                            end
                            
                        endcase
                        
                    end else begin
                        next_state [S_RUN] = 1;                     
                    end
                end
                
                current_state [S_RET] : begin
                    next_state[S_NOP_RUN] = 1'b1;
                    ctl_cmd_next_PC [OP_ADDR_PC_RESTORE_FROM_STACK] = 1;
                end
                
                current_state [S_RUN_EXT] : begin
                    ctl_pipeline_resume = 1;
                end
                
                current_state [S_NOP_RUN] : begin
                    ctl_cmd_next_PC [OP_ADDR_NOP] = 1;
                    next_state[S_RUN] = 1'b1;
                end
                        
                default: begin
                    next_state[S_RUN] = 1'b1;
                end
                
            endcase
              
        end : state_machine_comb
            
endmodule : fast_core_ALU

`default_nettype wire
