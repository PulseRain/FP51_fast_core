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
// SRT_Radix4_cascade
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

module SRT_Radix4_cascade #(parameter DATA_WIDTH) (
        
    //=======  clock and reset ======
        input  wire                                        clk,
        input  wire                                        reset_n,
        
    //========== INPUT ==========
        input  wire signed [DATA_WIDTH * 2 - 1 : 0]        Q_accum_in,
        input  wire unsigned [DATA_WIDTH - 1 : 0]          divisor,
        input  wire unsigned [DATA_WIDTH - 1 + 3 : 0]      partial_remainder_in,
        
    //========== OUTPUT ==========
        output wire signed [DATA_WIDTH * 2 - 1: 0]         Q_accum_out,
        output wire  signed [DATA_WIDTH - 1 + 3 : 0]       partial_remainder_out    
        
    //========== IN/OUT ==========
);
    
     wire  signed [DATA_WIDTH * 2 - 1 : 0]      Q_accum_out_1st_stage;
     wire  signed [DATA_WIDTH - 1 + 3 : 0]      partial_remainder_out_1st_stage;
     
        
    /* SRT_Radix4_single #(.DATA_WIDTH (DATA_WIDTH)) SRT_Radix4_single_1st (
                    .*,
                    .Q_accum_in (Q_accum_in),
                    .divisor (divisor),
                    .partial_remainder_in (partial_remainder_in),
                    
                    .q_out (),
                    .Q_accum_out (Q_accum_out_1st_stage),
                    .partial_remainder_out (partial_remainder_out_1st_stage));
     
     SRT_Radix4_single #(.DATA_WIDTH (DATA_WIDTH)) SRT_Radix4_single_2nd (
                    .*,
                    .Q_accum_in (Q_accum_out_1st_stage),
                    .divisor (divisor),
                    .partial_remainder_in (partial_remainder_out_1st_stage),
                    
                    .q_out (),
                    .Q_accum_out (Q_accum_out),
                    .partial_remainder_out (partial_remainder_out));
*/   
     
     SRT_Radix4_single #(.DATA_WIDTH (DATA_WIDTH)) SRT_Radix4_single_i (
                    .*,
                    .Q_accum_in (Q_accum_in),
                    .divisor (divisor),
                    .partial_remainder_in (partial_remainder_in),
                    
                    .q_out (),
                    .Q_accum_out (Q_accum_out),
                    .partial_remainder_out (partial_remainder_out));
     
    
endmodule : SRT_Radix4_cascade

`default_nettype wire
