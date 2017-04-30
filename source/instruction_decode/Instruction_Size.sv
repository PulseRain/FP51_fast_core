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
//    determine instruction size (1, 2 or 3 bytes) 
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"

`default_nettype none

module Instruction_Size (
    
        //========== INPUT ==========

        input wire                          clk,                             // clock input, 80 MHZ
        input wire                          reset_n,                         // reset, active low
        
        input wire  unsigned [7 : 0]        instruction_byte,

        //========== OUTPUT ==========
        
        output wire unsigned [1 : 0]        size_of_instruction
        
);
    
    wire single_byte_no_branch_indicator;
    wire two_byte_no_branch_indicator;
    
    is_single_byte_instruction one_byte_i (.*,
            .instruction_byte (instruction_byte),
            .single_byte_no_branch_indicator (single_byte_no_branch_indicator)
    );
    
    is_two_byte_instruction two_byte_i (.*,
            .instruction_byte (instruction_byte),
            .two_byte_no_branch_indicator (two_byte_no_branch_indicator)
    );
    
    assign size_of_instruction = {~single_byte_no_branch_indicator, ~two_byte_no_branch_indicator};
    
endmodule : Instruction_Size

`default_nettype wire
