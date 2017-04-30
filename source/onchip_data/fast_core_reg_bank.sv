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
//      register bank
//=============================================================================


`include "fast_core_common.svh"

`default_nettype none


//-----------------------------------------------------------------------------
// fast_core_reg_bank_single
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      single register bank for R0/R1
//
// See Also:
//    
//-----------------------------------------------------------------------------

module fast_core_reg_bank_single 
        #(parameter BANK_INDEX = 0)(
        
        //========== INPUT ==========
        input wire                          clk,                            
        input wire                          reset_n,                        
        
        input wire  unsigned [15 : 0]       addr,
        input wire  unsigned [7 : 0]        data_in,
        
        input wire                          we,
    
        //========== OUTPUT ==========
        output logic unsigned [7 : 0]       R0,
        output logic unsigned [7 : 0]       R1      
    
        
);
    always_ff @(posedge clk, negedge reset_n) begin
        if (!reset_n) begin
            R0 <= 0;
            R1 <= 0;
        end else begin
            if (we && (addr [4 : 3] == 2'(BANK_INDEX)) && (!addr[15:5]) && (!addr[2 : 1]) && (addr[0])) begin
                R1 <= data_in;
            end
            
            if (we && (addr [4 : 3] == 2'(BANK_INDEX)) && (!addr[15:5]) && (!addr[2 : 1]) && (!addr[0])) begin
                R0 <= data_in;
            end
        end
        
    end
    
endmodule : fast_core_reg_bank_single


//-----------------------------------------------------------------------------
// fast_core_reg_bank
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      register bank for R0/R1 (4 sets)
//
// See Also:
//    
//-----------------------------------------------------------------------------

module fast_core_reg_bank (

        //========== INPUT ==========
        input wire                          clk,                             
        input wire                          reset_n,                         
        
        input wire  unsigned [1 : 0]        active_bank_index,
        
        input wire  unsigned [15 : 0]       addr,
        input wire  unsigned [7 : 0]        data_in,
        
        input wire                          we,
    
        //========== OUTPUT ==========
        output logic unsigned [7 : 0]       R0,
        output logic unsigned [7 : 0]       R1      
);
    
    wire unsigned [7 : 0] R0_out_0, R1_out_0;
    wire unsigned [7 : 0] R0_out_1, R1_out_1;
    wire unsigned [7 : 0] R0_out_2, R1_out_2;
    wire unsigned [7 : 0] R0_out_3, R1_out_3;
    
     
    fast_core_reg_bank_single #(.BANK_INDEX(0)) bank_0(.*,
            .addr (addr),
            .data_in (data_in),
            .we (we),
            
            .R0 (R0_out_0),
            .R1 (R1_out_0));
    
    fast_core_reg_bank_single #(.BANK_INDEX(1)) bank_1(.*,
            .addr (addr),
            .data_in (data_in),
            .we (we),
            
            .R0 (R0_out_1),
            .R1 (R1_out_1));
    
    
    fast_core_reg_bank_single #(.BANK_INDEX(2)) bank_2(.*,
            .addr (addr),
            .data_in (data_in),
            .we (we),
            
            .R0 (R0_out_2),
            .R1 (R1_out_2));
    
    
    fast_core_reg_bank_single #(.BANK_INDEX(3)) bank_3(.*,
            .addr (addr),
            .data_in (data_in),
            .we (we),
            
            .R0 (R0_out_3),
            .R1 (R1_out_3));
    
    
    always_comb begin
                
        case (active_bank_index) // synthesis parallel_case 
            2'b01 : begin
                R0   = R0_out_1;
                R1   = R1_out_1;
            end
            
            2'b10 : begin
                R0   = R0_out_2;
                R1   = R1_out_2;
            end
            
            2'b11 : begin
                R0   = R0_out_3;
                R1   = R1_out_3;
            end
            
            default : begin
                R0   = R0_out_0;
                R1   = R1_out_0;
            end
            
        endcase
    end

endmodule : fast_core_reg_bank

`default_nettype wire
