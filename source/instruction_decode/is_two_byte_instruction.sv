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
//    determine if the instruction is 2 byte long and if it is 
// a branch instruction
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"

`default_nettype none

module is_two_byte_instruction (
    
        //========== INPUT ==========

        input wire                          clk,                             // clock input, 80 MHZ
        input wire                          reset_n,                         // reset, active low
        
        input wire [7 : 0]                  instruction_byte,
        
        //========== OUTPUT ==========
        output logic                        two_byte_no_branch_indicator
        
);
    
    always_comb begin
        if (    (instruction_byte == INST_ADD_A_DIR) ||
                (instruction_byte == INST_ADD_A_DATA) ||
                (instruction_byte == INST_ADDC_A_DIR) ||
                (instruction_byte == INST_ADDC_A_DATA) ||
                (instruction_byte == INST_ANL_A_DIR) ||
                (instruction_byte == INST_ANL_A_DATA) ||
                (instruction_byte == INST_ANL_DIR_A) ||
                (instruction_byte == INST_ANL_C_BIT) ||
                (instruction_byte == INST_ANL_C_NBIT) ||
                (instruction_byte == INST_CLR_BIT) ||
                (instruction_byte == INST_CPL_BIT) ||
                (instruction_byte == INST_DEC_DIR) ||
                (instruction_byte [7 : 3] == INST_DJNZ_R_REL_MSB) ||
                (instruction_byte == INST_INC_DIR) ||
                (instruction_byte == INST_SJMP) ||
                (instruction_byte == INST_JC) ||
                (instruction_byte == INST_JNC) ||
                (instruction_byte == INST_JNZ) ||
                (instruction_byte == INST_JZ) ||
                (instruction_byte == INST_MOV_A_DIR) ||
                (instruction_byte == INST_MOV_A_DATA) ||
                (instruction_byte [7 : 3] == INST_MOV_R_DIR_MSB) ||
                (instruction_byte [7 : 3] == INST_MOV_R_DATA_MSB) ||
                (instruction_byte == INST_MOV_DIR_A) ||
                (instruction_byte [7 : 3] == INST_MOV_DIR_R_MSB) ||
                (instruction_byte [7 : 1] == INST_MOV_DIR_AT_R_MSB) ||
                (instruction_byte [7 : 1] == INST_MOV_AT_R_DIR_MSB) ||
                (instruction_byte [7 : 1] == INST_MOV_AT_R_DATA_MSB) ||
                (instruction_byte == INST_MOV_C_BIT) ||
                (instruction_byte == INST_MOV_BIT_C) ||
                (instruction_byte == INST_ORL_A_DIR) ||
                (instruction_byte == INST_ORL_A_DATA) ||
                (instruction_byte == INST_ORL_DIR_A) ||
                (instruction_byte == INST_ORL_C_BIT) ||
                (instruction_byte == INST_ORL_C_NBIT) ||
                (instruction_byte == INST_POP) ||
                (instruction_byte == INST_PUSH) ||
                (instruction_byte == INST_SETB_BIT) ||
                (instruction_byte == INST_SUBB_A_DIR) ||
                (instruction_byte == INST_SUBB_A_DATA) ||
                (instruction_byte == INST_XCH_A_DIR) ||
                (instruction_byte == INST_XRL_A_DIR) ||
                (instruction_byte == INST_XRL_A_DATA) ||
                (instruction_byte == INST_XRL_DIR_A) || 
                (instruction_byte [4 : 0] == INST_ACALL_LSB) ||
                (instruction_byte [4 : 0] == INST_AJMP_LSB) ) begin
                
            two_byte_no_branch_indicator = 1'b1;    
        end else begin
            two_byte_no_branch_indicator = 1'b0;
        end
                
    end         

endmodule : is_two_byte_instruction 
    
`default_nettype wire
