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
//    RAM (both internal and external) for FP51. For FP51, there is physically
// no difference between internal and external RAM. The term 
// internal / external is only meaningful to traditional 8051.
//=============================================================================

`include "common.svh"
`include "fast_core_common.svh"

`default_nettype none


//-----------------------------------------------------------------------------
// fast_core_RAM
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      infer memory
//
// See Also:
//    
//-----------------------------------------------------------------------------


module fast_core_RAM (
        
        //========== INPUT ==========
        input wire                      clk,
        input wire                      reset_n,
        
        input wire unsigned [15 : 0]    read_addr,
        input wire unsigned [15 : 0]    write_addr,
        
        input wire unsigned [7 : 0]     data_in,
        input wire                      we,
        
        //========== OUTPUT ==========
        output logic unsigned [7 : 0]   data_out
        
);
    logic [0:ON_CHIP_DATA_RAM_SIZE_IN_BYTES - 1][7 : 0] mem  /* synthesis ramstyle = "M9K" */  ;
    
    always_ff @(posedge clk) begin
        if (we) begin
            mem[write_addr[$clog2(ON_CHIP_DATA_RAM_SIZE_IN_BYTES) - 1 : 0]][7 : 0] <= data_in;
        end
        
        data_out <= mem [read_addr[$clog2(ON_CHIP_DATA_RAM_SIZE_IN_BYTES) - 1 : 0]][7 : 0];
    end
    
endmodule : fast_core_RAM

`default_nettype wire
