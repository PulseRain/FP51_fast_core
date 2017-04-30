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
//    determine if the instruction is 1 byte long and if it is 
// a branch instruction
//=============================================================================


`include "common.svh"
`include "MCS_51_Instructions.svh"

`default_nettype none

module is_single_byte_instruction (
    
        //========== INPUT ==========

        input wire                          clk,                             
        input wire                          reset_n,                        
        
        input wire [7 : 0]                  instruction_byte,
        
        //========== OUTPUT ==========
        output logic                        single_byte_no_branch_indicator
        
);
    
    always_comb begin
        if ( (instruction_byte[7:3] == INST_ADD_A_R_MSB) ||
             (instruction_byte[7:1] == INST_ADD_A_AT_R_MSB) ||
             
             (instruction_byte[7:3] == INST_ADDC_A_R_MSB) ||
             (instruction_byte[7:1] == INST_ADDC_A_AT_R_MSB) ||
             
             (instruction_byte[7:3] == INST_ANL_A_R_MSB) ||
             (instruction_byte[7:1] == INST_ANL_A_AT_R_MSB) ||
             
             (instruction_byte == INST_CLR_A) ||
             (instruction_byte == INST_CLR_C) ||
             (instruction_byte == INST_CPL_A) ||
             (instruction_byte == INST_CPL_C) ||
             
             (instruction_byte == INST_DA_A) ||
             (instruction_byte == INST_DEC_A) ||
             (instruction_byte[7:3] == INST_DEC_R_MSB) ||
             (instruction_byte[7:1] == INST_DEC_AT_R_MSB) ||
             
             (instruction_byte == INST_DIV_AB) ||
             (instruction_byte == INST_INC_A) ||
             (instruction_byte[7:3] == INST_INC_R_MSB) ||
             (instruction_byte[7:1] == INST_INC_AT_R_MSB) ||
             (instruction_byte == INST_INC_DPTR) ||
             
             (instruction_byte[7:3] == INST_MOV_A_R_MSB) ||
             (instruction_byte[7:1] == INST_MOV_A_AT_R_MSB) ||
             
             (instruction_byte[7:3] == INST_MOV_R_A_MSB) ||
             (instruction_byte[7:1] == INST_MOV_AT_R_A_MSB) ||
             
             (instruction_byte == INST_MOVC_A_DPTR) ||
             (instruction_byte == INST_MOVC_A_PC) ||
             
             (instruction_byte[7:1] == INST_MOVX_A_AT_R_MSB) ||
             (instruction_byte == INST_MOVX_A_AT_DPTR) ||
             
             (instruction_byte[7:1] == INST_MOVX_AT_R_A_MSB) ||
             (instruction_byte == INST_MOVX_AT_DPTR_A) ||
             (instruction_byte == INST_MUL_AB) ||
             (instruction_byte == INST_NOP) ||
             
             (instruction_byte[7:3] == INST_ORL_A_R_MSB) ||
             (instruction_byte[7:1] == INST_ORL_A_AT_R_MSB) ||
             
             (instruction_byte == INST_RL_A) ||
             (instruction_byte == INST_RLC_A) ||
             (instruction_byte == INST_RR_A) ||
             (instruction_byte == INST_RRC_A) ||
             (instruction_byte == INST_SETB_C) ||
             
             (instruction_byte == INST_SETB_C) ||
             (instruction_byte[7:3] == INST_SUBB_A_R_MSB) ||
             (instruction_byte[7:1] == INST_SUBB_A_AT_R_MSB) ||
             
             (instruction_byte == INST_SWAP_A) ||
             (instruction_byte[7:3] == INST_XCH_A_R_MSB) ||
             (instruction_byte[7:1] == INST_XCH_A_AT_R_MSB) ||
             (instruction_byte[7:1] == INST_XCHD_A_AT_R_MSB) ||
             
             (instruction_byte[7:3] == INST_XRL_A_R_MSB) ||
             (instruction_byte[7:1] == INST_XRL_A_AT_R_MSB) ||
             (instruction_byte == INST_RET) ||
             (instruction_byte == INST_RETI) ) begin
             
             single_byte_no_branch_indicator = 1'b1;
             
        end else begin
            single_byte_no_branch_indicator = 1'b0;
        end
        
    end

    

    /* == clock rate test
    
    always_ff @(posedge clk, negedge reset_n) begin
        if (!reset_n) begin
            single_byte_no_branch_indicator <= 1'b0;
            instruction_byte <= 0;
        end else begin
            instruction_byte <= instruction_byte_tmp;

            if ( (instruction_byte[7:3] == INST_ADD_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_ADD_A_AT_R_MSB) ||
                 
                 (instruction_byte[7:3] == INST_ADDC_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_ADDC_A_AT_R_MSB) ||
                 
                 (instruction_byte[7:3] == INST_ANL_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_ANL_A_AT_R_MSB) ||
                 
                 (instruction_byte == INST_CLR_A) ||
                 (instruction_byte == INST_CLR_C) ||
                 (instruction_byte == INST_CPL_A) ||
                 (instruction_byte == INST_CPL_C) ||
                 
                 (instruction_byte == INST_DA_A) ||
                 (instruction_byte == INST_DEC_A) ||
                 (instruction_byte[7:3] == INST_DEC_R_MSB) ||
                 (instruction_byte[7:1] == INST_DEC_AT_R_MSB) ||
                 
                 (instruction_byte == INST_DIV_AB) ||
                 (instruction_byte[7:3] == INST_INC_R_MSB) ||
                 (instruction_byte[7:1] == INST_INC_AT_R_MSB) ||
                 (instruction_byte == INST_INC_DPTR) ||
                 
                 (instruction_byte[7:3] == INST_MOV_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_MOV_A_AT_R_MSB) ||
                 
                 (instruction_byte[7:3] == INST_MOV_R_A_MSB) ||
                 (instruction_byte[7:1] == INST_MOV_AT_R_A_MSB) ||
                 
                 (instruction_byte == INST_MOVC_A_DPTR) ||
                 (instruction_byte == INST_MOVC_A_PC) ||
                 
                 (instruction_byte[7:1] == INST_MOVX_A_AT_R_MSB) ||
                 (instruction_byte == INST_MOVX_A_DPTR) ||
                 
                 (instruction_byte[7:1] == INST_MOVX_AT_R_A_MSB) ||
                 (instruction_byte == INST_MOVX_AT_DPTR_A) ||
                 (instruction_byte == INST_MUL_AB) ||
                 (instruction_byte == INST_NOP) ||
                 
                 (instruction_byte[7:3] == INST_ORL_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_ORL_A_AT_R_MSB) ||
                 
                 (instruction_byte == INST_RL_A) ||
                 (instruction_byte == INST_RLC_A) ||
                 (instruction_byte == INST_RR_A) ||
                 (instruction_byte == INST_RRC_A) ||
                 (instruction_byte == INST_SETB_C) ||
                 
                 (instruction_byte == INST_SETB_C) ||
                 (instruction_byte[7:3] == INST_SUBB_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_SUBB_A_AT_R_MSB) ||
                 
                 (instruction_byte == INST_SWAP_A) ||
                 (instruction_byte[7:3] == INST_XCH_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_XCH_A_AT_R_MSB) ||
                 (instruction_byte[7:1] == INST_XCHD_A_AT_R_MSB) ||
                 
                 (instruction_byte[7:3] == INST_XRL_A_R_MSB) ||
                 (instruction_byte[7:1] == INST_XRL_A_AT_R_MSB) ) begin
                 
                 single_byte_no_branch_indicator <= 1'b1;
                 
            end else begin
                single_byte_no_branch_indicator <= 1'b0;
            end
            
        end
        
        
    end
*/
    
endmodule : is_single_byte_instruction 
    
`default_nettype wire
