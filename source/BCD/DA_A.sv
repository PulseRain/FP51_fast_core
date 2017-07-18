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
//  [1] 8051 Instruction Set Manual, DA
//      http://www.keil.com/support/man/docs/is51/is51_da.htm
//=============================================================================

`default_nettype none

//-----------------------------------------------------------------------------
// DA_A
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      Decimal Adjust
//
// See Also:
//      Ref [1]
//-----------------------------------------------------------------------------

module DA_A (
      
        //=======  clock and reset ======
        input  wire                                         clk,
        input  wire                                         reset_n,
        
        //========== INPUT ==========
        input  wire                                         enable_in,
        input  wire                                         CY,
        input  wire                                         AC,
        input  wire unsigned [7 : 0]                        ACC,
        
        //========== OUTPUT ==========
        output wire                                         c_flag,
        output wire unsigned [7 : 0]                        sum
        //========== IN/OUT ==========
);      
        
        logic   unsigned [8 : 0]                            sum_i;
        logic                                               enable_in_d1;
        
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                enable_in_d1 <= 0;
            end else begin
                enable_in_d1 <= enable_in;
            end
        end
        
            
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                sum_i <= 0;
            end else if (enable_in) begin
                if ((ACC[3:0] > 9) || (AC)) begin
                    sum_i <= ACC + 8'h6;
                end else begin
                    sum_i <= ACC;
                end
            end else if (enable_in_d1) begin    
                if ((sum_i [7 : 4] > 9) || (CY)) begin
                    sum_i [8 : 4] <= {1'b0, sum_i [7 : 4]} + 5'h6;
                end else begin
                    sum_i [8 : 4] <= {1'b0, sum_i [7 : 4]};
                end
            end
        end 
        
    /*  always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                c_flag <= 0;
            end else begin
                c_flag <= CY | sum_i [8];
            end
        end
    */  
        assign sum = sum_i [7 : 0];
        assign c_flag = CY | sum_i [8];
        
endmodule : DA_A

`default_nettype wire
