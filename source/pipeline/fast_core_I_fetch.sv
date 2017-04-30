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
//    Two way instruction fetch for FP51 fast core. 
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// fast_core_I_fetch
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

module fast_core_I_fetch (

        //========== INPUT ==========

        input wire                                  clk,           // clock input                
        input wire                                  reset_n,       // reset, active low

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
    
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // read enable
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        /*always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                re_A <= 0;
                re_B <= 0;
            end else begin
                re_A <= ctl_fetch_instruction_request;
                re_B <= ctl_fetch_instruction_request;
            end
        end
    */
        always_comb begin
            re_A = ctl_fetch_instruction_request;
            re_B = ctl_fetch_instruction_request;
        end
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Fetch Address A  
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {OP_ADDR_A_PC_PLUS_3, OP_ADDR_A_ADJUST} op_addr_A = 0;
                
        localparam ADDR_A_NUM_OF_OPERATIONS = op_addr_A.num();
        logic [ADDR_A_NUM_OF_OPERATIONS - 1:0]  ctl_addr_A_cmd;
        logic unsigned [PC_BITWIDTH - 1 : 0]    fetch_addr_A_save;
        logic                                   ctl_save_fetch_addr_A;
                        
                
        // op_addr_A cast for debug, one-hot translation, 
        // enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "op_addr_A" variable
                
        // synthesis translate_off
        ///////////////////////////////////////////////////////////////////////
        always_comb begin : op_addr_A_cast_for_debug
            for (int i = 0; i < ADDR_A_NUM_OF_OPERATIONS; ++i) begin
                if (ctl_addr_A_cmd[i]) begin
                    $cast(op_addr_A, i);
                end
            end
        end : op_addr_A_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on   
                
                
        // fetch address A
                            
        always_comb begin : fetch_addr_A_proc
            case (1'b1) // synthesis parallel_case 
                        
                ctl_addr_A_cmd[OP_ADDR_A_ADJUST] : begin
                    fetch_addr_A = fetch_addr_A_save + addr_adjust;
                end
                                                
                ctl_addr_A_cmd[OP_ADDR_A_PC_PLUS_3] : begin
                    fetch_addr_A = PC + ($size(PC))'(3);
                end
                                                                            
                default : begin 
                    fetch_addr_A = fetch_addr_A_save;
                end
                
            endcase
        end : fetch_addr_A_proc
                
        always_ff @(posedge clk, negedge reset_n) begin : fetch_addr_A_save_proc
            if (!reset_n) begin
                fetch_addr_A_save <= 0;
            end else if (ctl_save_fetch_addr_A) begin
                fetch_addr_A_save <= fetch_addr_A;
            end
                    
        end : fetch_addr_A_save_proc
                
                
                    
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // Fetch Address B  
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        enum {OP_ADDR_B_PC, OP_ADDR_B_PLUS_3} op_addr_B = 0;
        
        localparam ADDR_B_NUM_OF_OPERATIONS = op_addr_B.num();
        logic [ADDR_B_NUM_OF_OPERATIONS - 1:0] ctl_addr_B_cmd;
        logic unsigned [PC_BITWIDTH - 1 : 0]    fetch_addr_B_save;
        
                
        // op_addr_B cast for debug, one-hot translation, 
        // enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "op_addr_B" variable
                
        // synthesis translate_off
        ///////////////////////////////////////////////////////////////////////
        always_comb begin : op_addr_B_cast_for_debug
            for (int i = 0; i < ADDR_B_NUM_OF_OPERATIONS; ++i) begin
                if (ctl_addr_B_cmd[i]) begin
                    $cast(op_addr_B, i);
                end
            end
        end : op_addr_B_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on   
                    
        
        // fetch address B
        always_comb begin : fetch_addr_B_proc
            case (1'b1) // synthesis parallel_case 
                            
                ctl_addr_B_cmd[OP_ADDR_B_PC] : begin
                    fetch_addr_B = PC;
                end
                
                ctl_addr_B_cmd[OP_ADDR_B_PLUS_3] : begin
                    fetch_addr_B = fetch_addr_B_save + ($size(fetch_addr_B_save))'(3);
                end
                
                        
                default : begin 
                    fetch_addr_B = fetch_addr_B_save;
                end
                
            endcase
                
        end : fetch_addr_B_proc
                
        always_ff @(posedge clk, negedge reset_n) begin : fetch_addr_B_save_proc
            if (!reset_n) begin
                fetch_addr_B_save <= 0;
            end else begin
                fetch_addr_B_save <= fetch_addr_B;
            end
                    
        end : fetch_addr_B_save_proc
                
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // FSM
        //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                    
        enum {S_HOLD, S_RUN} states = 0;
                    
        localparam FSM_NUM_OF_STATES = states.num();
        logic [FSM_NUM_OF_STATES - 1:0] current_state = 0, next_state = 0;
                    
        // Declare states
        always_ff @(posedge clk, negedge reset_n) begin : state_machine_reg
            if (!reset_n) begin
                current_state <= 0;
            end else if (sync_reset) begin
                current_state <= ($size(current_state))'(1 << S_HOLD);
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
                
            ctl_addr_A_cmd = 0;
            ctl_addr_B_cmd = 0;
                            
            ctl_save_fetch_addr_A = 0;
            
            case (1'b1) // synthesis parallel_case 
                                                    
                    
                current_state[S_HOLD]: begin
                        
                    ctl_addr_A_cmd[OP_ADDR_A_PC_PLUS_3] = 1;
                    ctl_addr_B_cmd[OP_ADDR_B_PC] = 1;
                    ctl_save_fetch_addr_A = 1;
                        
                    if (ctl_fetch_instruction_request) begin
                        next_state [S_RUN] = 1;
                    end else begin
                        next_state [S_HOLD] = 1;    
                    end
                            
                end
                        
                current_state [S_RUN] : begin
                        
                    ctl_save_fetch_addr_A = 1;
                    next_state [S_RUN] = 1; 
                        
                    if (!ctl_fetch_instruction_request) begin

                    end else begin
                        
                            
                        // fetch address A
                        ctl_addr_A_cmd[OP_ADDR_A_ADJUST] = 1;
                                
                            
                        // fetch address B
                        ctl_addr_B_cmd[OP_ADDR_B_PLUS_3] = 1'b1;
                    
                    end
                end
                                                    
                default: begin
                    next_state[S_HOLD] = 1'b1;
                end
                    
            endcase
                  
        end : state_machine_comb

endmodule : fast_core_I_fetch
    
`default_nettype wire
