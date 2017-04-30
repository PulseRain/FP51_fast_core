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
//    FP51 fast core, 1T instruction cycle. 
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// FP51_fast_core
//
// Parameters:
//    FOR_SIM: when set, it will load instructions from simulated instruction ROM
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

module FP51_fast_core #(parameter FOR_SIM = 0) (
        
    //=======================================================================
    // clock / reset
    //=======================================================================
    
        input wire                                  clk,                             // clock input
        input wire                                  reset_n,                         // reset, active low
        
    //=======================================================================
    // instruction memory external r/w 
    //=======================================================================
        
        input  wire                                 inst_mem_we,
        input  wire unsigned [PC_BITWIDTH - 3 : 0]  inst_mem_wr_addr,
        input  wire unsigned [31 : 0]               inst_mem_data_in,
        
        input  wire                                 inst_mem_re,
        input  wire unsigned [PC_BITWIDTH - 1 : 0]  inst_mem_re_addr,
                    
        output wire                                 inst_mem_re_enable_out,
        output logic unsigned [31 : 0]              inst_mem_data_out,
        
    //=======================================================================
    // interrupt, interface with interrupt controller 
    //=======================================================================
        
        input wire                                  int_gen,
        input wire unsigned [7 : 0]                 int_addr,
        output wire                                 interrupt_return,

    //=======================================================================
    // Wishbone Host Interface 
    //=======================================================================
        output wire                                 WB_RD_CYC_O,
        output wire                                 WB_RD_STB_O,
        output wire  unsigned [DATA_WIDTH - 1 : 0]  WB_RD_ADR_O,
        input  wire  unsigned [DATA_WIDTH - 1 : 0]  WB_RD_DAT_I,
        input  wire                                 WB_RD_ACK_I,
        
        output wire                                 WB_WR_CYC_O,
        output wire                                 WB_WR_STB_O,
        output wire                                 WB_WR_WE_O,
        output wire unsigned [DATA_WIDTH - 1 : 0]   WB_WR_ADR_O,
        output wire unsigned [DATA_WIDTH - 1 : 0]   WB_WR_DAT_O,
        input  wire                                 WB_WR_ACK_I,
        
    //=======================================================================
    // ports 
    //=======================================================================
            
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P0,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P1,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P2,
        inout wire unsigned [DATA_WIDTH - 1 : 0]    P3,
        
    //=======================================================================
    // debug 
    //=======================================================================
        input wire                                  pause,
        input wire                                  break_on,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   break_addr_A,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   break_addr_B,
        input wire                                  run_pulse,
        
        output logic                                debug_stall,
        output logic unsigned [PC_BITWIDTH - 1 : 0] debug_PC,
        
        input wire                                  debug_data_read,
        input wire                                  debug_rd_indirect1_direct0,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   debug_data_read_addr,
        input wire                                  debug_data_read_restore,
        
        input wire                                  debug_data_write,
        input wire unsigned [PC_BITWIDTH - 1 : 0]   debug_data_write_addr,
        input wire unsigned                         debug_wr_indirect1_direct0,
        input wire unsigned [DATA_WIDTH - 1 : 0]    debug_data_write_data,
        
        
        output logic                                debug_read_data_enable_out,
        output logic unsigned [DATA_WIDTH - 1 : 0]  debug_read_data_out 
    );
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        wire                                        ctl_fetch_instruction_request;
        wire   unsigned [23 : 0]                    IR;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       PC;
        logic  unsigned [1 : 0]                     addr_adjust;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       fetch_addr_A;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       fetch_addr_B;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       Instruction_mem_addr_B;
                
        wire                                        I_mem_enable_out_A;
        wire                                        I_mem_enable_out_B;
            
        wire    unsigned [23 : 0]                   I_mem_data_out_A;
        wire    unsigned [23 : 0]                   I_mem_data_out_B;
        logic   unsigned [23 : 0]                   I_mem_data_out_B_reg;
                
        wire    unsigned [7 : 0]                    XPAGE;
        wire    unsigned [7 : 0]                    R0, R1;
        wire    unsigned [1 : 0]                    active_reg_bank_index;

        wire    unsigned [1 : 0]                    prefetch_size_adjust;
            
        wire                                        ctl_ALU_adjust_a;
        wire                                        ctl_ALU_adjust_b;
        wire                                        ctl_ALU_data_a;
        wire                                        ctl_ALU_data_b;
        wire                                        ctl_PSW_flag_update;
        wire                                        ctl_load_A;
            
        wire    unsigned [7 : 0]                    ALU_adjust_a, ALU_adjust_b;
        wire    unsigned [7 : 0]                    data_rd_address, data_wr_address;
        
        wire    unsigned [15 : 0]                   xdata_rd_address, xdata_wr_address;
                
        wire                                        ctl_indirect_read, ctl_indirect_write;
        
        wire    [ALU_NUM_OF_OPERATIONS - 1 : 0]     ctl_ALU_cmd;
        wire    [PC_NUM_OF_OPERATIONS - 1:0]        ctl_PC_cmd;
        wire                                        ctl_ALU_active;
        
        wire                                        SFR_we;
        wire    unsigned [7 : 0]                    onchip_data_out;
        
        wire    unsigned [7 : 0]                    PSW_value;
        wire                                        CY_current, AC_current;
        wire    unsigned [7 : 0]                    ACC, B;
        
        wire    unsigned [7 : 0]                    ALU_op_a, ALU_op_b;
        
        wire    unsigned [15 : 0]                   ALU_reg_data_wr_address;
        
        wire                                        ALU_reg_ctl_indirect_write; 
        wire                                        ALU_reg_ctl_PSW_flag_update;
        wire                                        ALU_reg_ctl_load_A;
        wire                                        ALU_reg_ctl_RL_A;
        wire                                        ALU_reg_ctl_RLC_A;
        wire                                        ALU_reg_ctl_RR_A;
        wire                                        ALU_reg_ctl_RRC_A;
       
        wire                                        onchip_data_we;
        wire    unsigned [7 : 0]                    onchip_data_to_write;
        
        wire                                        ctl_pipeline_hold;
        wire                                        ctl_pipeline_resume;
        
        wire                                        ctl_write_enable;
        
        wire                                        post_control_indirect_R0_R1_access_flag;
        wire                                        preALU_indirect_R0_R1_access_flag;
        wire                                        ALU_indirect_R0_R1_access_flag;
        wire    [23 : 0]                            ALU_IR;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       ALU_PC;
        
        
        wire    [ALU_NUM_OF_OPERATIONS - 1 : 0]     post_control_reg_ctl_ALU_cmd;
        wire    unsigned [15 : 0]                   post_control_reg_data_wr_address;
        wire    unsigned [15 : 0]                   post_control_reg_data_rd_address;
                    
        wire                                        post_control_reg_ctl_indirect_write;
        wire                                        post_control_reg_ctl_PSW_flag_update;
        wire                                        post_control_reg_ctl_load_A;
        wire    unsigned [7 : 0]                    post_control_reg_ALU_adjust_a;
        wire    unsigned [7 : 0]                    post_control_reg_ALU_adjust_b;
        
        wire                                        post_control_reg_ctl_ALU_adjust_a;
        wire                                        post_control_reg_ctl_ALU_adjust_b;
        
        wire                                        post_control_reg_ctl_ALU_data_a;
        wire                                        post_control_reg_ctl_ALU_data_b;
        
        wire   unsigned [23 : 0]                    post_control_reg_IR;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       post_control_reg_PC;
        wire                                        ctl_post_control_active_out;
            
        wire                                        post_control_reg_ctl_xread_enable;
        wire                                        post_control_reg_ctl_xwrite_enable;
                
        wire    [ALU_NUM_OF_OPERATIONS - 1 : 0]     preALU_reg_ctl_ALU_cmd;
        wire    unsigned [15 : 0]                   preALU_reg_data_wr_address;
        wire    unsigned [15 : 0]                   preALU_reg_data_rd_address;
        wire                                        preALU_reg_ctl_indirect_write;
        wire                                        preALU_reg_ctl_PSW_flag_update;
        wire                                        preALU_reg_ctl_load_A;
        wire    unsigned [7 : 0]                    preALU_reg_ALU_adjust_a;
        wire    unsigned [7 : 0]                    preALU_reg_ALU_adjust_b;
        
        wire                                        preALU_reg_ctl_ALU_adjust_a;
        wire                                        preALU_reg_ctl_ALU_adjust_b;
        
        wire                                        preALU_reg_ctl_ALU_data_a;
        wire                                        preALU_reg_ctl_ALU_data_b;
        
        wire                                        preALU_ctl_RL_A;
        wire                                        preALU_ctl_RLC_A;
        wire                                        preALU_ctl_RR_A;
        wire                                        preALU_ctl_RRC_A;
        
        wire   unsigned [23 : 0]                    preALU_reg_IR;
        wire   unsigned [PC_BITWIDTH - 1 : 0]       preALU_reg_PC;
        wire                                        ctl_preALU_active_out;
        
        wire                                        preALU_reg_ctl_xread_enable;
        wire                                        preALU_reg_ctl_xwrite_enable;
        
        wire unsigned [7 : 0]                       preALU_bit_mask;
        wire                                        preALU_reg_ctl_ALU_bit_mask_a;      
                    
        wire                                        instruction_mem_re_A, instruction_mem_re_B;
        wire                                        post_control_reg_ctl_write_enable;
        wire                                        preALU_reg_ctl_write_enable;
        
        wire                                        CY_next, AC_next, P_next, OV_next;
        
        wire  unsigned [15 : 0]                     DPTR;
        
        wire  unsigned [15 : 0]                     MOVC_addr;
        logic unsigned [15 : 0]                     MOVC_addr_d1;
        wire                                        ctl_load_MOVC;
        logic unsigned [3 : 0]                      ctl_load_MOVC_sr;
        
        wire  unsigned [7 : 0]                      onchip_mem_data_in;
        
        wire                                        ctl_xread_enable;
        wire                                        ctl_xwrite_enable;
        
        wire  unsigned [15 : 0]                     SP;
                        
        wire                                        post_control_reg_ctl_ALU_bit_mask_a;
        wire unsigned [BIT_SHIFT_WIDTH - 1 : 0]     post_control_reg_bit_mask_shift;
        wire                                        post_control_reg_ctl_bit_mask_invert;           
        
        wire                                        post_control_ctl_bit_load;
        wire                                        post_control_ctl_inv_bit_load;
        
        wire                                        preALU_ctl_bit_load;
        wire                                        preALU_ctl_inv_bit_load;
        
        wire                                        post_control_ctl_CY_update;
        wire                                        preALU_ctl_CY_update;
        wire                                        ALU_ctl_CY_update;
                
        wire                                        post_control_SP_write_lock;
        wire                                        post_control_DPTR_write_lock;
        
        wire                                        preALU_SP_write_lock;
        wire                                        preALU_DPTR_write_lock;
        
        wire                                        ALU_SP_write_lock;
        wire                                        ALU_DPTR_write_lock;
        
        wire                                        ctl_DPTR_load_data;
        wire                                        ctl_DPTR_inc;
        wire                                        ctl_DPTR_read;
        wire                                        ctl_DPTR_write;
        
        wire  unsigned [SP_INC_BITS - 1 : 0]        ctl_stack_move_amount;
        wire                                        ctl_stack_read;
        wire                                        ctl_stack_write;
        
        wire                                        post_control_reg_ctl_DPTR_load_data;
        wire                                        post_control_reg_ctl_DPTR_inc;
        wire                                        post_control_reg_ctl_DPTR_read;
        wire                                        post_control_reg_ctl_DPTR_write;
        
        wire  unsigned [SP_INC_BITS - 1 : 0]        post_control_reg_ctl_stack_move_amount;
        wire                                        post_control_reg_ctl_stack_read;
        wire                                        post_control_reg_ctl_stack_write;
        
        wire                                        preALU_reg_ctl_DPTR_load_data;
        wire                                        preALU_reg_ctl_DPTR_inc;
        wire                                        preALU_reg_ctl_DPTR_read;
        wire                                        preALU_reg_ctl_DPTR_write;
        
        wire  unsigned [SP_INC_BITS - 1 : 0]        preALU_reg_ctl_stack_move_amount;
        wire                                        preALU_reg_ctl_stack_read;
        wire                                        preALU_reg_ctl_stack_write;
        wire                                        preAUL_reg_ctl_SFR_active_read;
        wire                                        preAUL_reg_ctl_SFR_active_write;
                
        wire                                        ctl_interruptable;
        wire                                        post_control_reg_ctl_interruptable;
        wire                                        preALU_reg_ctl_interruptable;
        
        
        wire                                        ALU_reg_ctl_DPTR_load_data;
        wire                                        ALU_reg_ctl_DPTR_inc;
        wire                                        ALU_reg_ctl_DPTR_read;
        wire                                        ALU_reg_ctl_DPTR_write;
        wire  unsigned [SP_INC_BITS - 1 : 0]        ALU_reg_ctl_stack_move_amount;
        wire                                        ALU_reg_ctl_stack_read;
        wire                                        ALU_reg_ctl_stack_write;        
        
        wire                                        pipeline_stall;
        wire                                        reg_jump_active;
        
        wire                                        post_control_reg_ctl_indirect_read;
        
        wire                                        ctl_reset_I_fetch;
        wire unsigned [PC_BITWIDTH - 1 : 0]         next_PC;
        wire unsigned [DATA_WIDTH - 1 : 0]          stack_read_data;
        
        logic unsigned [DATA_WIDTH - 1 : 0]         P0_meta, P1_meta, P2_meta, P3_meta;
        logic unsigned [DATA_WIDTH - 1 : 0]         P0_in, P1_in, P2_in, P3_in;
        
                
        wire   unsigned [DATA_WIDTH - 1 : 0]        P0_direction;
        wire   unsigned [DATA_WIDTH - 1 : 0]        P1_direction;
        wire   unsigned [DATA_WIDTH - 1 : 0]        P2_direction;
        wire   unsigned [DATA_WIDTH - 1 : 0]        P3_direction;
        
        wire   unsigned [DATA_WIDTH - 1 : 0]        P0_out, P1_out, P2_out, P3_out;
        logic  unsigned [DATA_WIDTH - 1 : 0]        P0_out_d1, P1_out_d1, P2_out_d1, P3_out_d1;
                
    
        wire                                        ALU_int_gen_flag;
        wire unsigned [7 : 0]                       ALU_int_addr_reg;
        
        wire unsigned [1 : 0]                       size_of_instruction;
        wire unsigned [1 : 0]                       post_control_reg_size_of_instruction;
        wire unsigned [1 : 0]                       preALU_reg_size_of_instruction;
        wire unsigned [1 : 0]                       ALU_reg_size_of_instruction;
        wire unsigned [PC_BITWIDTH - 1 : 0]         INT_PC;
        
        logic                                       inst_re_A_reg, inst_re_B_reg;
        
        logic   unsigned [PC_BITWIDTH - 1 : 0]      fetch_addr_A_reg, fetch_addr_B_reg;
        
        logic   unsigned [3 : 0]                    inst_mem_re_sr;
        
        wire    unsigned [31 : 0]                   mem_data_out_B;
        
        logic   unsigned [1 : 0]                    debug_data_read_sr;
                
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Instruction Memory
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
        fast_core_I_fetch fast_core_I_fetch_i (.*,
            .sync_reset (ctl_reset_I_fetch),
            .ctl_fetch_instruction_request (ctl_fetch_instruction_request),
            .current_instruction (IR),
            .PC (PC),
            .addr_adjust (addr_adjust),
            
            .re_A (instruction_mem_re_A),
            .re_B (instruction_mem_re_B),
            
            .fetch_addr_A (fetch_addr_A),
            .fetch_addr_B (fetch_addr_B)
        );
                
        assign Instruction_mem_addr_B = inst_mem_re ? inst_mem_re_addr : 
               (ctl_load_MOVC_sr[0] ? MOVC_addr_d1 : fetch_addr_B);
            
        Instruction_Memory #(.FOR_SIM (FOR_SIM)) I_Memory_i (.*,
            .we (inst_mem_we),
            .wr_addr (inst_mem_wr_addr),
            .data_in (inst_mem_data_in),
            
            .re_A (inst_re_A_reg),
            .re_B (inst_re_B_reg),
                            
            .re_addr_A (fetch_addr_A_reg),
            .re_addr_B (fetch_addr_B_reg),
                        
            .enable_out_A (I_mem_enable_out_A),
            .enable_out_B (I_mem_enable_out_B),
            
            .instruction_out_A (I_mem_data_out_A),
            .instruction_out_B (I_mem_data_out_B),
            .mem_data_out_B (mem_data_out_B)
        );
            
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                inst_re_A_reg <= 0;
                inst_re_B_reg <= 0;
                fetch_addr_A_reg <= 0;
                fetch_addr_B_reg <= 0;
                inst_mem_re_sr <= 0;
                inst_mem_data_out <= 0;
            end else begin
                inst_re_A_reg <= instruction_mem_re_A;
                inst_re_B_reg <= instruction_mem_re_B | ctl_load_MOVC_sr[0];
                
                fetch_addr_A_reg <= fetch_addr_A;
                fetch_addr_B_reg <= Instruction_mem_addr_B;
                
                inst_mem_re_sr <= {inst_mem_re_sr [$high (inst_mem_re_sr) - 1 : 0], inst_mem_re};
                
                if (inst_mem_re_sr[2]) begin
                    inst_mem_data_out <= mem_data_out_B;
                end
                
            end
            
        end
            
        assign inst_mem_re_enable_out = inst_mem_re_sr[3];
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                I_mem_data_out_B_reg <= 0;
            end else if (I_mem_enable_out_B) begin
                I_mem_data_out_B_reg <= I_mem_data_out_B;
            end
        end
                        
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                ctl_load_MOVC_sr <= 0;
                MOVC_addr_d1 <= 0;
            end else begin
                MOVC_addr_d1 <= MOVC_addr;
                ctl_load_MOVC_sr <= {ctl_load_MOVC_sr [$high(ctl_load_MOVC_sr) - 1 : 0], ctl_load_MOVC};
            end
        end         
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // onchip data
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        //assign onchip_mem_data_in = ctl_load_MOVC ? I_mem_data_out_B_reg [23 : 16] : onchip_data_to_write;
                    
        fast_core_onchip_data fast_core_onchip_data_i (.*,
             .peripheral_data_in (WB_RD_DAT_I),
             
             .P0_in (P0_in),
             .P1_in (P1_in),
             .P2_in (P2_in),
             .P3_in (P3_in),
                             
             .ctl_stack_read (post_control_reg_ctl_stack_read),
             .ctl_stack_write (ALU_reg_ctl_stack_write),
                
            .read_addr (post_control_reg_data_rd_address),
            .write_addr (ALU_reg_data_wr_address),
            .data_in (onchip_data_to_write),
            .we (onchip_data_we),
            .data_for_ACC_load (ALU_op_b),
            .wr_indirect1_direct0 (ALU_reg_ctl_indirect_write),
            .rd_indirect1_direct0 (post_control_reg_ctl_indirect_read),
                        
            .ctl_stack_move_amount (ALU_reg_ctl_stack_move_amount),
                    
            .CY (CY_next),
            .AC (AC_next),
            .P (P_next),
            .OV (OV_next),
            
            .ctl_CY_update (ALU_ctl_CY_update),
            
            .MOVC_data_in (I_mem_data_out_B_reg [23 : 16]),
            .ctl_load_MOVC_data (ctl_load_MOVC_sr[3] & ctl_preALU_active_out),
                
            .ctl_PSW_flag_update (ALU_reg_ctl_PSW_flag_update),
            .ctl_load_A (ALU_reg_ctl_load_A),
            
            .ctl_RLC_A (ALU_reg_ctl_RLC_A),
            .ctl_RRC_A (ALU_reg_ctl_RRC_A),
            
            .ctl_DPTR_update (ALU_reg_ctl_DPTR_load_data),
            .DPTR_data (ALU_IR [15 : 0]),
            .ctl_DPTR_inc (ALU_reg_ctl_DPTR_inc),
            
            .SFR_we (SFR_we),
            .data_out (onchip_data_out),
            .R0 (R0),
            .R1 (R1),
            
            .ACC (ACC),
            .B (B),
            .PSW (PSW_value),
            .P0 (P0_out),
            .P1 (P1_out),
            .P2 (P2_out),
            .P3 (P3_out),
            .PCON (),
            .DPH (DPTR [15 : 8]),
            .DPL (DPTR [7 : 0]),
            
            .P0_direction (P0_direction),
            .P1_direction (P1_direction),
            .P2_direction (P2_direction),
            .P3_direction (P3_direction),
                        
            .SP (SP),
            .XPAGE (XPAGE)
        );
        
        assign CY_current = PSW_value[PSW_CY_INDEX];
        assign AC_current = PSW_value[PSW_AC_INDEX];
        assign active_reg_bank_index = PSW_value [PSW_RS1_INDEX : PSW_RS0_INDEX];
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Wishbone
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        assign WB_RD_CYC_O = 1'b1;
        assign WB_RD_STB_O = 1'b1;
        assign WB_RD_ADR_O = post_control_reg_data_rd_address [DATA_WIDTH - 1 : 0];
        
        assign WB_WR_CYC_O = 1'b1;
        assign WB_WR_STB_O = 1'b1;
        assign WB_WR_WE_O  = SFR_we;
        assign WB_WR_ADR_O = ALU_reg_data_wr_address [DATA_WIDTH - 1 : 0];
        assign WB_WR_DAT_O = onchip_data_to_write;
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Controller
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        fast_core_control fast_core_control_i (.*,
            .Prog_Mem_enable_in (I_mem_enable_out_A),
            .Prog_Mem_data_in_A (I_mem_data_out_A),
            .Prog_Mem_data_in_B (I_mem_data_out_B),
            
            .pipeline_stall (pipeline_stall | debug_stall),
            .reg_jump_active (reg_jump_active),
            
            .int_gen_flag (ALU_int_gen_flag & (~debug_stall)),
            .int_addr_reg (ALU_int_addr_reg),
            
            .next_PC (next_PC),         
            
            .DPTR (DPTR),
            .R0 (R0),
            .R1 (R1),
            .active_reg_bank_index (active_reg_bank_index),
            
            .ctl_pipeline_hold (ctl_pipeline_hold),
            .ctl_pipeline_resume (ctl_pipeline_resume), 
            
            .post_control_indirect_R0_R1_access_flag (post_control_indirect_R0_R1_access_flag),
            .post_control_IR (post_control_reg_IR),
            
            .preALU_indirect_R0_R1_access_flag (preALU_indirect_R0_R1_access_flag),
            .preALU_IR (preALU_reg_IR),
            .ALU_indirect_R0_R1_access_flag (ALU_indirect_R0_R1_access_flag),
            .ALU_IR (ALU_IR),
                    
            .ctl_fetch_instruction_request (ctl_fetch_instruction_request),
            
            .ctl_ALU_adjust_a (ctl_ALU_adjust_a),
            .ctl_ALU_adjust_b (ctl_ALU_adjust_b),
            .ctl_ALU_data_a (ctl_ALU_data_a),
            .ctl_ALU_data_b (ctl_ALU_data_b),
            
            .ctl_PSW_flag_update (ctl_PSW_flag_update),
            .ctl_load_A (ctl_load_A),
                        
            .ALU_adjust_a (ALU_adjust_a),
            .ALU_adjust_b (ALU_adjust_b),
            
            .data_rd_address (data_rd_address),
            .data_wr_address (data_wr_address),
            
            .xdata_rd_address (xdata_rd_address),
            .xdata_wr_address (xdata_wr_address),
            
            .ctl_indirect_read (ctl_indirect_read),
            .ctl_indirect_write (ctl_indirect_write),
            
            .ctl_ALU_cmd (ctl_ALU_cmd),
            .ctl_PC_cmd (ctl_PC_cmd),
            .IR (IR),
            .PC (PC),
            .prefetch_size_adjust (addr_adjust),
            .size_of_instruction (size_of_instruction),
            .ctl_ALU_active (ctl_ALU_active),
            .ctl_write_enable (ctl_write_enable),
            
            .ctl_xread_enable (ctl_xread_enable),
            .ctl_xwrite_enable (ctl_xwrite_enable),
        
            .ctl_DPTR_load_data (ctl_DPTR_load_data),
            .ctl_DPTR_inc (ctl_DPTR_inc),
            .ctl_DPTR_read (ctl_DPTR_read),
            .ctl_DPTR_write (ctl_DPTR_write),
            .ctl_stack_move_amount (ctl_stack_move_amount),
            .ctl_stack_read (ctl_stack_read),
            .ctl_stack_write (ctl_stack_write),
            .ctl_reset_I_fetch (ctl_reset_I_fetch),
            .ctl_interruptable (ctl_interruptable)
        );
                    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // post control
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        fast_core_post_control fast_core_post_control_i (.*,
            .IR (IR),
            .PC (PC),
            
            .R0 (R0),
            .R1 (R1),
            .XPAGE (XPAGE),
            
            .reg_jump_active (reg_jump_active),
            .ctl_interruptable (ctl_interruptable),
            .int_gen_flag (ALU_int_gen_flag),
            .size_of_instruction (size_of_instruction),
            
            .DPTR (DPTR),
            
            .preALU_reg_IR (preALU_reg_IR),
            .preALU_reg_ctl_DPTR_load_data (preALU_reg_ctl_DPTR_load_data),
            .preALU_reg_ctl_DPTR_inc (preALU_reg_ctl_DPTR_inc),
            
            .ALU_reg_IR (ALU_IR),
            .ALU_reg_ctl_DPTR_load_data (ALU_reg_ctl_DPTR_load_data),
            .ALU_reg_ctl_DPTR_inc (ALU_reg_ctl_DPTR_inc),
                    
            .SP (SP),
            .preALU_reg_ctl_stack_move_amount (preALU_reg_ctl_stack_move_amount),
            .ALU_reg_ctl_stack_move_amount (ALU_reg_ctl_stack_move_amount),
                
            .preALU_SP_write_lock (preALU_SP_write_lock),
            .preALU_DPTR_write_lock (preALU_DPTR_write_lock),
            
            .ALU_SP_write_lock (ALU_SP_write_lock),
            .ALU_DPTR_write_lock (ALU_DPTR_write_lock),
            
            .ctl_DPTR_load_data (ctl_DPTR_load_data),
            .ctl_DPTR_inc (ctl_DPTR_inc),
            .ctl_DPTR_read (ctl_DPTR_read),
            .ctl_DPTR_write (ctl_DPTR_write),
            
            .ctl_stack_move_amount (ctl_stack_move_amount),
            .ctl_stack_read (ctl_stack_read),
            .ctl_stack_write (ctl_stack_write),
            
            .ctl_indirect_read (ctl_indirect_read),
                            
            .ALU_adjust_a (ALU_adjust_a),
            .ALU_adjust_b (ALU_adjust_b),
            
            .active_reg_bank_index (active_reg_bank_index),
            
            .ctl_pipeline_hold (ctl_pipeline_hold),
            .ctl_pipeline_resume (ctl_pipeline_resume),
            
            .preALU_indirect_R0_R1_access_flag (preALU_indirect_R0_R1_access_flag),
            .preALU_IR (preALU_reg_IR),
            .ALU_indirect_R0_R1_access_flag (ALU_indirect_R0_R1_access_flag),
            .ALU_IR (ALU_IR),
            
            .ctl_ALU_cmd (ctl_ALU_cmd),
            
            .ctl_xread_enable (ctl_xread_enable),
            .ctl_xwrite_enable (ctl_xwrite_enable),
        
            .ctl_write_enable (ctl_write_enable),
            .ctl_indirect_write (ctl_indirect_write),
            
            .data_wr_address (data_wr_address),
            .data_rd_address (data_rd_address),
            
            .xdata_wr_address (xdata_wr_address),
            .xdata_rd_address (xdata_rd_address),
        
            .ctl_ALU_active (ctl_ALU_active),
            .ctl_ALU_adjust_a (ctl_ALU_adjust_a),
            .ctl_ALU_adjust_b (ctl_ALU_adjust_b),
            
            .ctl_ALU_data_a (ctl_ALU_data_a),
            .ctl_ALU_data_b (ctl_ALU_data_b),
            
            .ctl_PSW_flag_update (ctl_PSW_flag_update),
            .ctl_load_A (ctl_load_A),
            
            .reg_ctl_ALU_cmd (post_control_reg_ctl_ALU_cmd),
            .reg_data_wr_address (post_control_reg_data_wr_address),
            .reg_data_rd_address (post_control_reg_data_rd_address),
                
            .reg_ctl_indirect_write (post_control_reg_ctl_indirect_write),
            .reg_ctl_PSW_flag_update (post_control_reg_ctl_PSW_flag_update),
            .reg_ctl_load_A (post_control_reg_ctl_load_A),
            .reg_ctl_write_enable (post_control_reg_ctl_write_enable),
            .reg_ALU_adjust_a (post_control_reg_ALU_adjust_a),
            .reg_ALU_adjust_b (post_control_reg_ALU_adjust_b),
            
            .reg_ctl_ALU_adjust_a (post_control_reg_ctl_ALU_adjust_a),
            .reg_ctl_ALU_adjust_b (post_control_reg_ctl_ALU_adjust_b),
            
            .reg_ctl_ALU_data_a (post_control_reg_ctl_ALU_data_a),
            .reg_ctl_ALU_data_b (post_control_reg_ctl_ALU_data_b),
            
            .reg_ctl_xread_enable (post_control_reg_ctl_xread_enable),
            .reg_ctl_xwrite_enable (post_control_reg_ctl_xwrite_enable),
            
            .reg_ctl_indirect_read (post_control_reg_ctl_indirect_read),
                    
            .reg_IR (post_control_reg_IR),
            .reg_PC (post_control_reg_PC),
            .reg_size_of_instruction (post_control_reg_size_of_instruction),
            .ctl_preALU_active_out (ctl_post_control_active_out),
            .indirect_R0_R1_access_flag (post_control_indirect_R0_R1_access_flag),
            
            .reg_ctl_ALU_bit_mask_a (post_control_reg_ctl_ALU_bit_mask_a),
            .reg_bit_mask_shift (post_control_reg_bit_mask_shift),
            .reg_ctl_bit_mask_invert (post_control_reg_ctl_bit_mask_invert),
        
            .ctl_bit_load (post_control_ctl_bit_load),
            .ctl_inv_bit_load (post_control_ctl_inv_bit_load),
            
            .ctl_CY_update (post_control_ctl_CY_update),
            
            .reg_ctl_DPTR_load_data    (post_control_reg_ctl_DPTR_load_data),
            .reg_ctl_DPTR_inc          (post_control_reg_ctl_DPTR_inc),
            .reg_ctl_DPTR_read         (post_control_reg_ctl_DPTR_read),
            .reg_ctl_DPTR_write        (post_control_reg_ctl_DPTR_write),
            .reg_ctl_stack_move_amount (post_control_reg_ctl_stack_move_amount),
            .reg_ctl_stack_read        (post_control_reg_ctl_stack_read),
            .reg_ctl_stack_write       (post_control_reg_ctl_stack_write),
            .pipeline_stall (pipeline_stall),
            .reg_ctl_interruptable (post_control_reg_ctl_interruptable),
            
            .debug_stall (debug_stall),
            .debug_data_read (debug_data_read),
            .debug_rd_indirect1_direct0 (debug_rd_indirect1_direct0),
            .debug_data_read_addr (debug_data_read_addr),
            .debug_data_read_restore (debug_data_read_restore)
        
        );
                    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // preALU
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        fast_core_pre_ALU pre_ALU (.*,
            .int_gen_flag (ALU_int_gen_flag),
            .IR (post_control_reg_IR),
            .PC (post_control_reg_PC),
            .size_of_instruction (post_control_reg_size_of_instruction),
            
            .pipeline_stall (pipeline_stall),
            .reg_jump_active (reg_jump_active),
            .ctl_interruptable (post_control_reg_ctl_interruptable),
            
            .ctl_DPTR_load_data (post_control_reg_ctl_DPTR_load_data),
            .ctl_DPTR_inc       (post_control_reg_ctl_DPTR_inc),
            .ctl_DPTR_read      (post_control_reg_ctl_DPTR_read),
            .ctl_DPTR_write     (post_control_reg_ctl_DPTR_write),
            
            .ctl_stack_move_amount (post_control_reg_ctl_stack_move_amount),
            .ctl_stack_read        (post_control_reg_ctl_stack_read),
            .ctl_stack_write       (post_control_reg_ctl_stack_write),
            .ctl_indirect_read     (post_control_reg_ctl_indirect_read),
        
            .ALU_adjust_a (post_control_reg_ALU_adjust_a),
            .ALU_adjust_b (post_control_reg_ALU_adjust_b),
            .active_reg_bank_index (active_reg_bank_index),
            
            .ctl_pipeline_hold (ctl_pipeline_hold),
            .ctl_pipeline_resume (ctl_pipeline_resume),
            
            .ctl_bit_load (post_control_ctl_bit_load),
            .ctl_inv_bit_load (post_control_ctl_inv_bit_load),
            .ctl_CY_update (post_control_ctl_CY_update),
            
            .ctl_ALU_bit_mask_a (post_control_reg_ctl_ALU_bit_mask_a),
            .ctl_bit_mask_invert (post_control_reg_ctl_bit_mask_invert),
            .bit_mask_shift (post_control_reg_bit_mask_shift),
                    
            .ctl_ALU_cmd (post_control_reg_ctl_ALU_cmd),
            
            .ctl_xread_enable (post_control_reg_ctl_xread_enable),
            .ctl_xwrite_enable (post_control_reg_ctl_xwrite_enable),
        
            .ctl_write_enable (post_control_reg_ctl_write_enable),
            .ctl_indirect_write (post_control_reg_ctl_indirect_write),
            .data_wr_address (post_control_reg_data_wr_address),
            .data_rd_address (post_control_reg_data_rd_address),
            
            .ctl_ALU_active (ctl_post_control_active_out),
            .ctl_ALU_adjust_a (post_control_reg_ctl_ALU_adjust_a),
            .ctl_ALU_adjust_b (post_control_reg_ctl_ALU_adjust_b),
            .ctl_ALU_data_a (post_control_reg_ctl_ALU_data_a),
            .ctl_ALU_data_b (post_control_reg_ctl_ALU_data_b),
            .ctl_PSW_flag_update (post_control_reg_ctl_PSW_flag_update),
            .ctl_load_A (post_control_reg_ctl_load_A),
            
            .reg_ctl_ALU_cmd (preALU_reg_ctl_ALU_cmd),
            .reg_data_wr_address (preALU_reg_data_wr_address),
            .reg_data_rd_address (preALU_reg_data_rd_address),
            .reg_ctl_indirect_write (preALU_reg_ctl_indirect_write),
            .reg_ctl_PSW_flag_update (preALU_reg_ctl_PSW_flag_update),
            .reg_ctl_load_A (preALU_reg_ctl_load_A),
            .reg_ctl_write_enable (preALU_reg_ctl_write_enable),
            .reg_ALU_adjust_a (preALU_reg_ALU_adjust_a),
            .reg_ALU_adjust_b (preALU_reg_ALU_adjust_b),
            
            .reg_ctl_ALU_adjust_a (preALU_reg_ctl_ALU_adjust_a),
            .reg_ctl_ALU_adjust_b (preALU_reg_ctl_ALU_adjust_b),
            
            .reg_ctl_ALU_data_a (preALU_reg_ctl_ALU_data_a),
            .reg_ctl_ALU_data_b (preALU_reg_ctl_ALU_data_b),
            
            .reg_ctl_xread_enable (preALU_reg_ctl_xread_enable),
            .reg_ctl_xwrite_enable (preALU_reg_ctl_xwrite_enable),
                        
            .ctl_RL_A  (preALU_ctl_RL_A),
            .ctl_RLC_A (preALU_ctl_RLC_A),
            .ctl_RR_A  (preALU_ctl_RR_A),
            .ctl_RRC_A (preALU_ctl_RRC_A),
            
            .reg_IR (preALU_reg_IR),
            .reg_PC (preALU_reg_PC),
            .reg_size_of_instruction (preALU_reg_size_of_instruction),
            .ctl_preALU_active_out (ctl_preALU_active_out),
            .indirect_R0_R1_access_flag (preALU_indirect_R0_R1_access_flag),
            
            .bit_mask (preALU_bit_mask),
            .reg_ctl_ALU_bit_mask_a (preALU_reg_ctl_ALU_bit_mask_a),
            .reg_ctl_bit_load (preALU_ctl_bit_load),
            .reg_ctl_inv_bit_load (preALU_ctl_inv_bit_load),
            .reg_ctl_CY_update (preALU_ctl_CY_update),
            
            .reg_ctl_DPTR_load_data    (preALU_reg_ctl_DPTR_load_data),
            .reg_ctl_DPTR_inc          (preALU_reg_ctl_DPTR_inc),
            .reg_ctl_DPTR_read         (preALU_reg_ctl_DPTR_read),
            .reg_ctl_DPTR_write        (preALU_reg_ctl_DPTR_write),
            .reg_ctl_stack_move_amount (preALU_reg_ctl_stack_move_amount),
            .reg_ctl_stack_read        (preALU_reg_ctl_stack_read),
            .reg_ctl_stack_write       (preALU_reg_ctl_stack_write),
                    
            .SP_write_lock (preALU_SP_write_lock),
            .DPTR_write_lock (preALU_DPTR_write_lock),
            .reg_ctl_interruptable (preALU_reg_ctl_interruptable)
        );
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // ALU
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        fast_core_ALU ALU_i (.*,
            
            .int_gen (int_gen & (~debug_stall)),
            .int_addr (int_addr),
                
            .DPTR (DPTR),
            .SP (SP),
                
            .R0 (R0),
            .R1 (R1),
            .active_reg_bank_index (active_reg_bank_index),
            .ctl_interruptable (preALU_reg_ctl_interruptable),
            
            .IR (preALU_reg_IR),
            .PC (preALU_reg_PC),
            .size_of_instruction (preALU_reg_size_of_instruction),
            
            .ctl_DPTR_load_data    (preALU_reg_ctl_DPTR_load_data),
            .ctl_DPTR_inc          (preALU_reg_ctl_DPTR_inc),
            .ctl_DPTR_read         (preALU_reg_ctl_DPTR_read),
            .ctl_DPTR_write        (preALU_reg_ctl_DPTR_write),
            .ctl_stack_move_amount (preALU_reg_ctl_stack_move_amount),
            .ctl_stack_read        (preALU_reg_ctl_stack_read),
            .ctl_stack_write       (preALU_reg_ctl_stack_write),
            
            .bit_mask (preALU_bit_mask),
            .ctl_ALU_bit_mask_a (preALU_reg_ctl_ALU_bit_mask_a),
            
            .ctl_bit_load (preALU_ctl_bit_load),
            .ctl_inv_bit_load (preALU_ctl_inv_bit_load),
            
            .ctl_CY_update (preALU_ctl_CY_update),
            
            .CY_current (CY_current),
            .AC_current (AC_current),
            .ACC (ACC),
            .PSW (PSW_value),
            .B (B),
            .ALU_adjust_a (preALU_reg_ALU_adjust_a),
            .ALU_adjust_b (preALU_reg_ALU_adjust_b),
            
            .data_rd_address (preALU_reg_data_rd_address),
            .data_wr_address (preALU_reg_data_wr_address),
        
            .ctl_ALU_cmd (preALU_reg_ctl_ALU_cmd),
            .onchip_data_in (onchip_data_out),
            
            .indirect_R0_R1_access_flag (preALU_indirect_R0_R1_access_flag),
            
            .SP_write_lock (preALU_SP_write_lock),
            .DPTR_write_lock (preALU_DPTR_write_lock),
            
            .ctl_write_enable (preALU_reg_ctl_write_enable),
            .ctl_indirect_write (preALU_reg_ctl_indirect_write),
            .ctl_ALU_active (ctl_preALU_active_out),
            
            .ctl_ALU_adjust_a (preALU_reg_ctl_ALU_adjust_a),
            .ctl_ALU_adjust_b (preALU_reg_ctl_ALU_adjust_b),
            .ctl_ALU_data_a (preALU_reg_ctl_ALU_data_a),
            .ctl_ALU_data_b (preALU_reg_ctl_ALU_data_b),
            .ctl_PSW_flag_update (preALU_reg_ctl_PSW_flag_update),
            .ctl_load_A (preALU_reg_ctl_load_A),
            
            .SFR_MOVC_data_in (8'd0),
            .SFR_ctl_load_MOVC_data (1'b0),
            
            .ALU_op_a (ALU_op_a),
            .ALU_op_b (ALU_op_b),
            
            .ctl_RL_A (preALU_ctl_RL_A),
            .ctl_RLC_A (preALU_ctl_RLC_A),
            .ctl_RR_A (preALU_ctl_RR_A),
            .ctl_RRC_A (preALU_ctl_RRC_A),

            .ctl_xread_enable (preALU_reg_ctl_xread_enable),
            .ctl_xwrite_enable (preALU_reg_ctl_xwrite_enable),
            
            .reg_indirect_R0_R1_access_flag (ALU_indirect_R0_R1_access_flag),
            
            .reg_data_wr_address (ALU_reg_data_wr_address),
            .reg_ctl_indirect_write (ALU_reg_ctl_indirect_write),
            .reg_ctl_PSW_flag_update (ALU_reg_ctl_PSW_flag_update),
            .reg_ctl_load_A (ALU_reg_ctl_load_A),
            .reg_ctl_RL_A (ALU_reg_ctl_RL_A),
            .reg_ctl_RLC_A (ALU_reg_ctl_RLC_A),
            .reg_ctl_RR_A (ALU_reg_ctl_RR_A),
            .reg_ctl_RRC_A (ALU_reg_ctl_RRC_A),
            .reg_IR (ALU_IR),
            .reg_PC (ALU_PC),
            .reg_size_of_instruction (ALU_reg_size_of_instruction),
            .reg_onchip_data_we (onchip_data_we),
            .onchip_data_to_write (onchip_data_to_write),
            .ctl_pipeline_hold (ctl_pipeline_hold),
            .ctl_pipeline_resume (ctl_pipeline_resume),
            .CY (CY_next),
            .AC (AC_next),
            .P (P_next),
            .OV (OV_next),
                        
            .MOVC_addr (MOVC_addr),
            .ctl_load_MOVC (ctl_load_MOVC),
            .reg_ctl_CY_update (ALU_ctl_CY_update),
            
                    
            .reg_ctl_DPTR_load_data    (ALU_reg_ctl_DPTR_load_data),
            .reg_ctl_DPTR_inc          (ALU_reg_ctl_DPTR_inc),
            .reg_ctl_DPTR_read         (ALU_reg_ctl_DPTR_read),
            .reg_ctl_DPTR_write        (ALU_reg_ctl_DPTR_write),
            .reg_ctl_stack_move_amount (ALU_reg_ctl_stack_move_amount),
            .reg_ctl_stack_read        (ALU_reg_ctl_stack_read),
            .reg_ctl_stack_write       (ALU_reg_ctl_stack_write),
                
            .reg_jump_active (reg_jump_active),
            
            .reg_SP_write_lock (ALU_SP_write_lock),
            .reg_DPTR_write_lock (ALU_DPTR_write_lock),
            .next_PC (next_PC),
            .stack_read_data (stack_read_data),
            .int_gen_flag (ALU_int_gen_flag),
            .int_addr_reg (ALU_int_addr_reg),
            .INT_PC (INT_PC),
            .reg_ret_int (interrupt_return));
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Ports
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
            always_ff @(posedge clk, negedge reset_n) begin
                if (!reset_n) begin
                    P0_meta <= 0;
                    P1_meta <= 0;
                    P2_meta <= 0;
                    P3_meta <= 0;
                    
                    P0_in <= 0;
                    P1_in <= 0;
                    P2_in <= 0;
                    P3_in <= 0;
                end else begin
                    P0_meta <= P0;
                    P1_meta <= P1;
                    P2_meta <= P2;
                    P3_meta <= P3;
                    
                    P0_in <= P0_meta;
                    P1_in <= P1_meta;
                    P2_in <= P2_meta;
                    P3_in <= P3_meta;
                end
            end
        
            always_ff @(posedge clk) begin
                P0_out_d1 <= P0_out;
                P1_out_d1 <= P1_out;
                P2_out_d1 <= P2_out;
                P3_out_d1 <= P3_out;
            end
            
            genvar port_i;
            generate 
                for (port_i = 0; port_i < DATA_WIDTH; port_i = port_i + 1) begin : port_gen
                    assign P0[port_i] = P0_direction [port_i] ? P0_out_d1 [port_i] : 1'bZ;
                    assign P1[port_i] = P1_direction [port_i] ? P1_out_d1 [port_i] : 1'bZ;
                    assign P2[port_i] = P2_direction [port_i] ? P2_out_d1 [port_i] : 1'bZ;
                    assign P3[port_i] = P3_direction [port_i] ? P3_out_d1 [port_i] : 1'bZ;
                    
                end : port_gen
            endgenerate
        
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // debug
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
            always_ff @(posedge clk, negedge reset_n) begin : debug_stall_proc
                if (!reset_n) begin
                    debug_stall <= 0;
                end else if ((((ALU_int_gen_flag) && ( ({8'd0, ALU_int_addr_reg} == break_addr_A) || ({8'd0, ALU_int_addr_reg} == break_addr_B) ) ) || 
                        ((reg_jump_active) && ((next_PC == break_addr_A) || (next_PC == break_addr_B) )) ||
                        (((PC + size_of_instruction) == break_addr_A) || ((PC + size_of_instruction) == break_addr_B) ) ) && break_on && (!debug_stall) ) begin
                        debug_stall <= 1'b1;
                end else if (debug_stall) begin
                        debug_stall <= (~(run_pulse)) & pause;
                end else begin
                        debug_stall <= pause & ctl_interruptable;
                end
            end : debug_stall_proc
                
            always_ff @(posedge clk, negedge reset_n) begin : debug_read_data_proc
                if (!reset_n) begin
                    debug_read_data_enable_out <= 0;
                    debug_data_read_sr         <= 0;
                    debug_read_data_out        <= 0;
                    debug_PC                   <= 0;
                end else begin
                    debug_data_read_sr         <= {debug_data_read_sr[$high(debug_data_read_sr) - 1 : 0], debug_data_read};
                    debug_read_data_enable_out <= debug_data_read_sr[1];
                    debug_read_data_out        <= onchip_data_out;
                    debug_PC                   <= PC;
                end
            end : debug_read_data_proc
                    
endmodule : FP51_fast_core

`default_nettype wire
