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
//    common definition and module prototypes that are specific to fast core 
//=============================================================================

`ifndef FAST_CORE_COMMON_SVH
`define FAST_CORE_COMMON_SVH

`include "common.svh"

// PC
typedef enum {OP_PC_SEQUENTIAL} op_PC_t;
op_PC_t op_PC;          
parameter PC_NUM_OF_OPERATIONS = op_PC.num();


// ALU
typedef enum {OP_ALU_ADD, OP_ALU_ADDC, OP_ALU_SUBC, OP_ALU_SUB, OP_ALU_SUB_NEG, OP_ALU_AND, OP_ALU_OR,
    OP_ALU_XOR, OP_ALU_A_DIRECT_PASS, OP_ALU_MUL, OP_ALU_MUL_2ND, OP_ALU_MUL_3RD,
    OP_ALU_DIV_START, OP_ALU_DIV_SAVE_A, OP_ALU_DIV_SAVE_B, OP_ALU_DA_START,
    OP_ALU_DA_SAVE, OP_ALU_DA_WR, OP_ALU_SWAP_A, OP_ALU_DPTR_DATA, 
    OP_ALU_MOVC_A_DPTR, OP_ALU_MOVC_A_PC, OP_ALU_XCHD, OP_ALU_MOV_BIT_C, 
    OP_ALU_SAVE_PC_PLUS_2_HIGH, OP_ALU_SAVE_PC_PLUS_2_LOW,
    OP_ALU_SAVE_PC_PLUS_3_HIGH, OP_ALU_SAVE_PC_PLUS_3_LOW,
    OP_ALU_SAVE_INT_PC_HIGH, OP_ALU_SAVE_INT_PC_LOW} op_ALU_t;

op_ALU_t op_ALU;
parameter ALU_NUM_OF_OPERATIONS = op_ALU.num();



// next PC
typedef enum {OP_ADDR_JMP_REL_ADDR_2_BYTE, OP_ADDR_JMP_REL_ADDR_3_BYTE,
       OP_ADDR_ACALL_AJMP, OP_ADDR_JMP_A_DPTR, OP_ADDR_LCALL_LJMP,
       OP_ADDR_PC_RESTORE_FROM_STACK, OP_ADDR_NOP} op_next_PC_t;
op_next_PC_t op_next_PC;
parameter OP_NEXT_PC_NUM_OF_OPERATIONS = op_next_PC.num();



extern module fast_core_reg_bank (

        input wire                          clk,                             
        input wire                          reset_n,                         
        
        input wire  unsigned [1 : 0]        active_bank_index,
        
        input wire  unsigned [15 : 0]       addr,
        input wire  unsigned [7 : 0]        data_in,
        
        input wire                          we,
    
        output logic unsigned [7 : 0]       R0,
        output logic unsigned [7 : 0]       R1      
                    
    
);

extern module fast_core_IRAM (
        input wire                      clk,
        input wire                      reset_n,
            
        input wire unsigned [7 : 0]     read_addr,
        input wire unsigned [7 : 0]     write_addr,
        
        input wire unsigned [7 : 0]     data_in,
        input wire                      we,
        
        output logic unsigned [7 : 0]   data_out
        
);


extern module fast_core_SFR (
        input wire                                      clk,                             // clock input, 80 MHZ
        input wire                                      reset_n,                         // reset, active low
        
        input wire unsigned [DATA_WIDTH - 1 : 0]        peripheral_data_in,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]        P0_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]        P1_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]        P2_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]        P3_in,
                
        input wire  unsigned [7 : 0]                    read_addr,
        input wire  unsigned [DATA_WIDTH - 1 : 0]       data_in,
        
        input wire  unsigned [DATA_WIDTH - 1 : 0]       data_for_ACC_load,
        
        input wire unsigned [SP_INC_BITS - 1 : 0]       ctl_stack_move_amount,
                
        input wire  unsigned [7 : 0]        write_addr,
        input wire                          we,
        
        input wire                          CY,
        input wire                          AC,
        input wire                          P,
        input wire                          OV,
        
        input wire                          ctl_CY_update,

        input wire unsigned [DATA_WIDTH - 1 : 0]    MOVC_data_in,
        input wire                          ctl_load_MOVC_data,
        
        input wire                          ctl_PSW_flag_update,
        input wire                          ctl_load_A,
        input wire                          ctl_RLC_A,
        input wire                          ctl_RRC_A,
        
        input wire                          ctl_DPTR_update,
        input wire  unsigned [DATA_WIDTH * 2 - 1 : 0]   DPTR_data,
        
        input wire                          ctl_DPTR_inc,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      data_out,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      ACC,
        output logic unsigned [DATA_WIDTH - 1 : 0]      B,
        output logic unsigned [DATA_WIDTH - 1 : 0]      PSW,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P0,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P1,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P2,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P3,
        output logic unsigned [DATA_WIDTH - 1 : 0]      PCON,
        output logic unsigned [DATA_WIDTH - 1 : 0]      DPH,
        output logic unsigned [DATA_WIDTH - 1 : 0]      DPL,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      P0_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P1_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P2_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P3_direction,
        
        output logic unsigned [DATA_WIDTH * 2 - 1 : 0]  SP,
        output wire unsigned [DATA_WIDTH - 1 : 0]       XPAGE
);
    
    
extern module fast_core_I_fetch (

        //========== INPUT ==========

        input wire                                  clk,                             // clock input, 80 MHZ
        input wire                                  reset_n,                         // reset, active low

        input wire                                  sync_reset,
        input wire                                  ctl_fetch_instruction_request,
        input wire unsigned [23 : 0]                current_instruction,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   PC,
        input wire unsigned [1 : 0]                 addr_adjust,
            
        //========== OUTPUT ==========
        output logic                                re_A,
        output logic                                re_B,
        output logic unsigned [PC_BITWIDTH - 1 : 0] fetch_addr_A,
        output logic unsigned [PC_BITWIDTH - 1 : 0] fetch_addr_B
);
    
extern module fast_core_ALU (
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
        
        input   wire unsigned [23 : 0]          IR,
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
    //==    input   wire                            ctl_SFR_active_read,
    //==    input   wire                            ctl_SFR_active_write,
                
        input   wire [ALU_NUM_OF_OPERATIONS - 1 : 0]    ctl_ALU_cmd,
        input   wire unsigned [7 : 0]           onchip_data_in,
    //  input   wire unsigned [7 : 0]           xdata_in,
        
        
        input   wire                            indirect_R0_R1_access_flag,
        
        input   wire                            SP_write_lock,
        input   wire                            DPTR_write_lock,
        
        input   wire                            ctl_write_enable,
        input   wire                            ctl_indirect_write,
        input   wire  unsigned [15 : 0]         data_rd_address,
        input   wire  unsigned [15 : 0]         data_wr_address,
        
    //  input   wire  unsigned [15 : 0]         xdata_rd_address,
    //  input   wire  unsigned [15 : 0]         xdata_wr_address,
        
        input   wire                            ctl_ALU_active,
        input   wire                            ctl_ALU_adjust_a,
        input   wire                            ctl_ALU_adjust_b,
        input   wire                            ctl_ALU_data_a,
        input   wire                            ctl_ALU_data_b,
        input   wire                            ctl_PSW_flag_update,
        input   wire                            ctl_load_A,
        
        input   wire  unsigned [7 : 0]          SFR_MOVC_data_in,
        input   wire                            SFR_ctl_load_MOVC_data,
        
        output  logic  unsigned [7 : 0]         ALU_op_a,
        output  logic  unsigned [7 : 0]         ALU_op_b,   
        
        output logic                            reg_indirect_R0_R1_access_flag,
        output logic   unsigned [15 : 0]            reg_data_wr_address,
    //    output logic   unsigned [15 : 0]      reg_xdata_wr_address,
        
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
        
        output logic                                    reg_SP_write_lock,
        output logic                                    reg_DPTR_write_lock,
        output logic unsigned [PC_BITWIDTH - 1 : 0]     next_PC,
        output logic unsigned [DATA_WIDTH - 1 : 0]      stack_read_data,
        output logic                                    int_gen_flag,
        output logic unsigned [7 : 0]                   int_addr_reg,
        output logic unsigned [PC_BITWIDTH - 1 : 0]     INT_PC,
        output logic                                    reg_ret_int,
        
        //===================================================================
        // Debug
        //===================================================================
        
        input wire                                      debug_data_write,
        input wire unsigned [PC_BITWIDTH - 1 : 0]       debug_data_write_addr,
        input wire unsigned                             debug_wr_indirect1_direct0,
        input wire unsigned [DATA_WIDTH - 1 : 0]        debug_data_write_data
        
);
    
extern module fast_core_pre_ALU (
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
    //  input   wire  unsigned [15 : 0]         xdata_wr_address,
    //  input   wire  unsigned [15 : 0]         xdata_rd_address,
        
        input   wire                            ctl_ALU_active,
        input   wire                            ctl_ALU_adjust_a,
        input   wire                            ctl_ALU_adjust_b,
        input   wire                            ctl_ALU_data_a,
        input   wire                            ctl_ALU_data_b,
        input   wire                            ctl_PSW_flag_update,
        input   wire                            ctl_load_A,
                
        output logic  [ALU_NUM_OF_OPERATIONS - 1 : 0]   reg_ctl_ALU_cmd,
        output logic   unsigned [15 : 0]        reg_data_wr_address,
        output logic   unsigned [15 : 0]        reg_data_rd_address,
    //  output logic   unsigned [15 : 0]        reg_xdata_wr_address,
    //  output logic   unsigned [15 : 0]        reg_xdata_rd_address,
        
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
    //==    output logic                            reg_ctl_SFR_active_read,
    //==    output logic                            reg_ctl_SFR_active_write,
                
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
    
extern module fast_core_post_control (
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
                
        output logic  [ALU_NUM_OF_OPERATIONS - 1 : 0]   reg_ctl_ALU_cmd,
        
        output logic   unsigned [15 : 0]        reg_data_wr_address,
        output logic   unsigned [15 : 0]        reg_data_rd_address,
    //  output logic   unsigned [15 : 0]        reg_xdata_wr_address,
    //  output logic   unsigned [15 : 0]        reg_xdata_rd_address,
        
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
    
extern module fast_core_control (
        input   wire                            clk,
        input   wire                            reset_n,
        
        input   wire                            Prog_Mem_enable_in,
        input   wire unsigned [23 : 0]          Prog_Mem_data_in_A,
        input   wire unsigned [23 : 0]          Prog_Mem_data_in_B,
        
        input   wire                            pipeline_stall,
        input   wire                            reg_jump_active,
        
        input   wire                            int_gen_flag,
        input   wire unsigned [7 : 0]           int_addr_reg,
                
        input   wire  unsigned [PC_BITWIDTH - 1 : 0]    next_PC,        
        
        input   wire unsigned [15 : 0]          DPTR,
        input   wire unsigned [7 : 0]           R0,
        input   wire unsigned [7 : 0]           R1,
        input   wire unsigned [1 : 0]           active_reg_bank_index,
        
        input   wire                            ctl_pipeline_hold,
        input   wire                            ctl_pipeline_resume,    
        
        input   wire                            post_control_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   post_control_IR,
        
        input   wire                            preALU_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   preALU_IR,
        input   wire                            ALU_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   ALU_IR,
    
        output logic                            ctl_fetch_instruction_request,
        output logic                            ctl_ALU_adjust_a,
        output logic                            ctl_ALU_adjust_b,
        output logic                            ctl_ALU_data_a,
        output logic                            ctl_ALU_data_b,
        output logic                            ctl_PSW_flag_update,
        output logic                            ctl_load_A,
                
        output logic unsigned [7 : 0]           ALU_adjust_a,
        output logic unsigned [7 : 0]           ALU_adjust_b,
        
        output logic unsigned [7 : 0]           data_rd_address,
        output logic unsigned [7 : 0]           data_wr_address,
        
        output logic unsigned [15 : 0]          xdata_rd_address,
        output logic unsigned [15 : 0]          xdata_wr_address,
        
        output logic                            ctl_indirect_read,
        output logic                            ctl_indirect_write,
        
        
        output logic [ALU_NUM_OF_OPERATIONS - 1 : 0]    ctl_ALU_cmd,
        output logic [PC_NUM_OF_OPERATIONS - 1:0]       ctl_PC_cmd,
        output logic unsigned [23 : 0]                  IR,
        output logic unsigned [PC_BITWIDTH - 1 : 0]     PC,
        output logic unsigned [1 : 0]                   prefetch_size_adjust,
        output wire  unsigned [1 : 0]                   size_of_instruction,
        output logic                                    ctl_ALU_active,
        output logic                                    ctl_write_enable,
        output logic                                    ctl_xread_enable,
        output logic                                    ctl_xwrite_enable,
        
        output logic                                    ctl_DPTR_load_data,
        output logic                                    ctl_DPTR_inc,
        output logic                                    ctl_DPTR_read,
        output logic                                    ctl_DPTR_write,
                
        output logic unsigned [SP_INC_BITS - 1 : 0]     ctl_stack_move_amount,
        output logic                                    ctl_stack_read,
        output logic                                    ctl_stack_write,
        output logic                                    ctl_reset_I_fetch,
        output logic                                    ctl_interruptable
        
);
    
extern module fast_core_onchip_data (
        input wire                      clk,
        input wire                      reset_n,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]    peripheral_data_in,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]    P0_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]    P1_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]    P2_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]    P3_in,
                    
        input wire                      ctl_stack_read,
        input wire                      ctl_stack_write,
        
        input wire unsigned [15 : 0]    read_addr,
        input wire unsigned [15 : 0]    write_addr,
        input wire unsigned [7 : 0]     data_in,
        input wire                      we,
        
        input wire  unsigned [7 : 0]    data_for_ACC_load,
        
        input wire                      wr_indirect1_direct0,
        input wire                      rd_indirect1_direct0,
        
        input wire unsigned [SP_INC_BITS - 1 : 0]   ctl_stack_move_amount,
        
        input wire                      CY,
        input wire                      AC,
        input wire                      P,
        input wire                      OV,
        

        input wire                      ctl_CY_update,
        input wire unsigned [7 : 0]     MOVC_data_in,
        input wire                      ctl_load_MOVC_data,
        
        input wire                      ctl_PSW_flag_update,
        input wire                      ctl_load_A,
        input wire                      ctl_RLC_A,
        input wire                      ctl_RRC_A,
        
        input wire                          ctl_DPTR_update,
        input wire  unsigned [15 : 0]       DPTR_data,
        
        input wire                          ctl_DPTR_inc,
        
        output wire                         SFR_we,
        output logic unsigned [7 : 0]       data_out,
        
        output wire unsigned [7 : 0]       R0,
        output wire unsigned [7 : 0]       R1,      
        
        
        output wire unsigned [7 : 0]        ACC,
        output wire unsigned [7 : 0]        B,
        output wire unsigned [7 : 0]        PSW,
        output wire unsigned [7 : 0]        P0,
        output wire unsigned [7 : 0]        P1,
        output wire unsigned [7 : 0]        P2,
        output wire unsigned [7 : 0]        P3,
        output wire unsigned [7 : 0]        PCON,
        output wire unsigned [7 : 0]        DPH,
        output wire unsigned [7 : 0]        DPL,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      P0_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P1_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P2_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P3_direction,
        
        output wire unsigned [15 : 0]                   SP,
        output wire unsigned [DATA_WIDTH - 1 : 0]       XPAGE
        
);
    
extern module Instruction_Memory  #(parameter FOR_SIM = 0) (

        //========== INPUT ==========

            input wire                          clk,                             // clock input, 80 MHZ
            input wire                          reset_n,                         // reset, active low
            
            
            
            input wire                                  we,
            input wire unsigned [PC_BITWIDTH - 3 : 0]   wr_addr,
            input wire unsigned [31 : 0]                data_in,
            
            input wire                                  re_A,
            input wire                                  re_B,
            
            input wire unsigned [PC_BITWIDTH - 1 : 0]   re_addr_A,
            input wire unsigned [PC_BITWIDTH - 1 : 0]   re_addr_B,
            
                                    
        //========== OUTPUT ==========
            output logic                                enable_out_A,
            output logic                                enable_out_B,
            
            output logic unsigned [23 : 0]              instruction_out_A,
            output logic unsigned [23 : 0]              instruction_out_B,
            output logic unsigned [31 : 0]              mem_data_out_B
            
);

`endif
