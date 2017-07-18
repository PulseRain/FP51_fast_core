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
// References:
//  [1] Building Embedded Systems: Programmable Hardware, 1st Edition, 
//      by Changyi Gu, 07/2016
//=============================================================================

`default_nettype none

//-----------------------------------------------------------------------------
// SRT_Radix4_division
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
//      Ref [1], Chapter 12
//-----------------------------------------------------------------------------

module SRT_Radix4_division #(parameter DATA_WIDTH = 8) (
      
    //=======  clock and reset ======
        input  wire                                         clk,
        input  wire                                         reset_n,
        
    //========== INPUT ==========
        input  wire                                         enable_in,
        input  wire unsigned [DATA_WIDTH - 1 : 0]           dividend,
        input  wire unsigned [DATA_WIDTH - 1 : 0]           divisor,
        
    //========== OUTPUT ==========
        output logic                                        ov_flag,  // overflow flag
        output logic unsigned [DATA_WIDTH - 1 : 0]          quotient,
        output logic unsigned [DATA_WIDTH - 1 : 0]          remainder
    //========== IN/OUT ==========
);

    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        logic unsigned [ $clog2(DATA_WIDTH / 2) : 0]    shift_counter;
        logic unsigned [DATA_WIDTH - 1 : 0]             divisor_reg;
        
        logic                                           ctl_load_shift_counter;
        
        logic signed [DATA_WIDTH - 1 + 3 : 0]           partial_remainder;
        
        logic                                           ctl_partial_remainder_update;
        logic                                           ctl_Q_accum_update;
        logic                                           ctl_load_result;
        
        logic signed [DATA_WIDTH * 2 - 1 : 0]           Q_accum;
        wire  signed [DATA_WIDTH * 2 - 1: 0]            Q_accum_out;
        wire  signed [DATA_WIDTH - 1 + 3 : 0]           partial_remainder_out;
        
        logic unsigned [$clog2(DATA_WIDTH)  : 0]        norm_shift;
        
        wire  signed [DATA_WIDTH * 2 - 1 : 0]           quotient_i;
        wire  unsigned [DATA_WIDTH - 1 : 0]             quotient_ii;
        wire  unsigned [DATA_WIDTH - 1 : 0]             remainder_i;
        logic unsigned [DATA_WIDTH - 1 : 0]             dividend_reg;
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // data path
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                shift_counter <= 0;
            end else if (ctl_load_shift_counter) begin
                shift_counter <= ($size(shift_counter))'(DATA_WIDTH / 2);
            end else if (shift_counter) begin
                shift_counter <= shift_counter - ($size(shift_counter))'(1);
            end
        end
        
        always_ff @(posedge clk, negedge reset_n) begin 
            if (!reset_n) begin
                divisor_reg <= 0;
                dividend_reg <= 0;
                ov_flag <= 0;
                norm_shift <= 0;
            end else if (enable_in) begin
                
                dividend_reg <= dividend;
                
                // normalize, not a general way to do it yet
                ov_flag <= 0;
                casex (divisor) // synthesis parallel_case 
                    8'b01??_???? : begin
                        norm_shift <= 4'd7;
                        divisor_reg <= {divisor [6 : 0], 1'b0};
                    end
                    
                    8'b001?_???? : begin
                        norm_shift <= 4'd6;
                        divisor_reg <= {divisor [5 : 0], 2'b00};
                    end
                    
                    8'b0001_???? : begin
                        norm_shift <= 4'd5;
                        divisor_reg <= {divisor [4 : 0], 3'b000};
                    end
                    
                    8'b0000_1??? : begin
                        norm_shift <= 4'd4;
                        divisor_reg <= {divisor [3 : 0], 4'b0000};
                    end
                    
                    8'b0000_01?? : begin
                        norm_shift <= 4'd3;
                        divisor_reg <= {divisor [2 : 0], 5'b0_0000};
                    end
                    
                    8'b0000_001? : begin
                        norm_shift <= 4'd2;
                        divisor_reg <= {divisor [1 : 0], 6'b00_0000};
                    end
                    
                    8'b0000_0001 : begin
                        norm_shift <= 4'd1;
                        divisor_reg <= {1'b1, 7'b000_0000};
                    end
                    
                    8'b1???_???? : begin
                        norm_shift <= 4'd8;
                        divisor_reg <= divisor;
                    end
                    
                    
                    default : begin
                        ov_flag <= 1'b1;
                        norm_shift <= 0;
                        divisor_reg <= 0;
                    end
                endcase
                                
            end
        end
        
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                Q_accum <= 0;
            end else if (enable_in) begin
                Q_accum <= 0;
            end else if (ctl_Q_accum_update) begin
                Q_accum <= Q_accum_out;
            end
        end
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                partial_remainder <= 0;
            end else if (enable_in) begin
                partial_remainder <= ($size(partial_remainder))'({(DATA_WIDTH)'(0), dividend});
            end else if (ctl_partial_remainder_update) begin
                partial_remainder <= partial_remainder_out;
            end
        end
    
        assign quotient_i = Q_accum_out >> norm_shift;
        assign quotient_ii = quotient_i [9 : 2];
        
        
        
        assign quotient = quotient_ii;
                       
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
        //      quotient <= 0;
                remainder <= 0;
            end else if (ctl_load_result) begin
        //      quotient <= quotient_ii;
                remainder <= dividend_reg - quotient_ii * divisor;
            end
        end
            
            
        
        SRT_Radix4_cascade #(.DATA_WIDTH (DATA_WIDTH)) SRT_Radix4_cascade_i (.*,
            .Q_accum_in (Q_accum),
            .divisor (divisor_reg),
            .partial_remainder_in (partial_remainder),
            .Q_accum_out (Q_accum_out),
            .partial_remainder_out (partial_remainder_out));
        
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_IDLE, S_RUN, S_DONE} states = S_IDLE;
                
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
            
            ctl_load_shift_counter = 0;
            
            ctl_partial_remainder_update = 0;
            
            ctl_Q_accum_update = 0;
            
            ctl_load_result = 0;
            
            case (1'b1) // synthesis parallel_case 
                
                current_state[S_IDLE]: begin
                    
                    ctl_load_shift_counter = 1'b1;
                    
                    if (enable_in) begin
                        next_state [S_RUN] = 1;
                    end else begin
                        next_state [S_IDLE] = 1;                        
                    end
                end
                
                current_state [S_RUN] : begin
                    ctl_partial_remainder_update = 1'b1;
                    ctl_Q_accum_update = 1'b1;
                    
                    if (shift_counter) begin
                        next_state [S_RUN] = 1;
                    end else begin
                        next_state [S_DONE] = 1;
                    end
                end
                        
                current_state [S_DONE] : begin
                    ctl_load_result = 1'b1;
                    next_state[S_IDLE] = 1'b1;
                end
                
                default: begin
                    next_state[S_IDLE] = 1'b1;
                end
                
            endcase
              
        end : state_machine_comb
        
endmodule : SRT_Radix4_division

`default_nettype wire
