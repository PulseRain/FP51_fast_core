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
// SRT_Radix4_single
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

module SRT_Radix4_single #(parameter DATA_WIDTH) (
      
    //=======  clock and reset ======
        input  wire                                        clk,
        input  wire                                        reset_n,
        
    //========== INPUT ==========
        input  wire signed [DATA_WIDTH * 2 - 1 : 0]        Q_accum_in,
        input  wire unsigned [DATA_WIDTH - 1 : 0]          divisor,
        input  wire signed [DATA_WIDTH - 1 + 3 : 0]        partial_remainder_in,
        
    //========== OUTPUT ==========
        output logic signed [2 : 0]                        q_out,
        output logic signed [DATA_WIDTH * 2 - 1 : 0]       Q_accum_out,
        output wire  signed [DATA_WIDTH - 1 + 3 : 0]       partial_remainder_out    
        
    //========== IN/OUT ==========
);

    logic   signed  [DATA_WIDTH - 1 + 4 : 0]            qd;
    wire    signed  [DATA_WIDTH  : 0]                   divisor_signed;
    wire    signed  [DATA_WIDTH  : 0]                   divisor_signed_neg;
    wire    signed  [DATA_WIDTH - 1 + 3 : 0]            partial_remainder_out_tmp;
    
    wire    unsigned [3 : 0]                            top_4bits;
    
    assign divisor_signed = {1'b0, divisor};
    assign divisor_signed_neg = ~divisor_signed + ($size(divisor_signed_neg))'(1);
    //assign qd = q_out * divisor_signed;
    
    
    always_comb begin
        case (q_out) // synthesis parallel_case
            
            3'b001 : begin
                qd = {{3{divisor_signed[$high(divisor_signed)]}}, divisor_signed};
            end
            
            3'b111 : begin
                qd = {{3{divisor_signed_neg[$high(divisor_signed_neg)]}}, divisor_signed_neg};
            end
            
            3'b010 : begin
                qd = {{3{divisor_signed[$high(divisor_signed)]}}, divisor_signed} 
                    + {{3{divisor_signed[$high(divisor_signed)]}}, divisor_signed};
            end
            
            3'b110 : begin
                qd = {{3{divisor_signed_neg[$high(divisor_signed_neg)]}}, divisor_signed_neg}
                    + {{3{divisor_signed_neg[$high(divisor_signed_neg)]}}, divisor_signed_neg};
            end
            
            3'b011 : begin
                qd = {{2{divisor_signed[$high(divisor_signed)]}}, divisor_signed, 1'b0} 
                    + {{3{divisor_signed[$high(divisor_signed)]}}, divisor_signed};;
            end
            
            3'b101 : begin
                qd = {{2{divisor_signed_neg[$high(divisor_signed_neg)]}}, divisor_signed_neg, 1'b0}
                    + {{3{divisor_signed_neg[$high(divisor_signed_neg)]}}, divisor_signed_neg};
            end
                        
            default : begin
                qd = 0;
            end
            
        endcase
                
    end
    
    assign partial_remainder_out_tmp = partial_remainder_in - qd [DATA_WIDTH - 1 + 3 : 0];
    
    assign partial_remainder_out = {partial_remainder_out_tmp [DATA_WIDTH : 0], 2'b00};
    
    assign Q_accum_out = {Q_accum_in [DATA_WIDTH * 2 - 3 : 0], 2'b00} 
                        + {{(DATA_WIDTH  * 2 - 3){q_out[2]}}, q_out};
    
    
    
//  always_ff @(posedge clk, negedge reset_n) begin
//      if (!reset_n) begin
//          partial_remainder_out <= 0;
//          Q_accum_out <= 0;
//      end else begin
//          partial_remainder_out <= {partial_remainder_out_tmp [DATA_WIDTH : 0], 2'b00};
//           Q_accum_out <= {Q_accum_in [DATA_WIDTH * 2 - 3 : 0], 2'b00} 
//                      + {{(DATA_WIDTH  * 2 - 3){q_out[2]}}, q_out};
//      end
//  end
    
    
    assign top_4bits = partial_remainder_in [DATA_WIDTH + 2 : DATA_WIDTH - 1];
    
    always_comb begin
        
        casex (top_4bits) // synthesis parallel_case 
            4'b0000 : begin
                q_out = 3'd0;
            end
            
            4'b1111 : begin
                q_out = -3'd0;
            end
            
            4'b0001 : begin
                q_out = 3'd1;
            end
            
            4'b1110 : begin
                q_out = -3'd1;
            end
                            
            4'b0010 : begin
                q_out = 3'd2;
            end
                
            4'b1101 : begin
                q_out = -3'd2;
            end
                            
            4'b0011 : begin
                if (divisor [DATA_WIDTH - 1 : DATA_WIDTH - 2] == 2'b11) begin
                    q_out = 3'd2;
                end else begin
                    q_out = 3'd3;
                end
            end
            
            4'b1100 : begin
                if (divisor [DATA_WIDTH - 1 : DATA_WIDTH - 2] == 2'b11) begin
                    q_out = -3'd2;
                end else begin
                    q_out = -3'd3;
                end
            end
            
            default : begin
                if (partial_remainder_in [$high(partial_remainder_in)]) begin
                    q_out = -3'd3;
                end else begin
                    q_out = 3'd3;
                end
            end
            
        endcase
        
    end
    
    
endmodule : SRT_Radix4_single

`default_nettype wire
