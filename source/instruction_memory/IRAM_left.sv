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
//    Instruction Memory (higher 16 bits)
//=============================================================================

`include "common.svh"

`default_nettype none

module IRAM_left ( 
             input wire clk,
             input wire we,
             input wire [15 : 0]  data_in,
             input wire [PC_BITWIDTH - 3 : 0] addr_a, addr_b,
             output logic [15:0] q_a, q_b);
            logic unsigned [15 : 0] rom[ON_CHIP_CODE_RAM_SIZE_IN_BYTES / 4 - 1 : 0] /* synthesis ramstyle = "M9K" */ ;
             
             
             always_ff @ (posedge clk) begin : rom_read_proc
                  if (we) begin
                    rom [addr_b] <= data_in;
                  end
                  
                  q_a <= rom[addr_a];
                  q_b <= rom[addr_b];
             end : rom_read_proc
             
endmodule : IRAM_left

`default_nettype wire
