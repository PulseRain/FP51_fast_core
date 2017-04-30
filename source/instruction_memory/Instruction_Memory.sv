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
//    Instruction Memory for FP51 fast core. 
//    use two bank, dual port memory to achieve two way fetch
//=============================================================================

`include "common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// Instruction_Memory
//
//
// Parameter:
//    FOR_SIM: set this param for simulation
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

module Instruction_Memory  #(parameter FOR_SIM = 0) (

        //========== INPUT ==========

            input wire                          clk,                             // clock input, 80 MHZ
            input wire                          reset_n,                         // reset, active low
            
            
            
            input wire                                  we,
            input wire unsigned [PC_BITWIDTH - 3 : 0]   wr_addr,
            input wire unsigned [31 : 0]                data_in,
            
            input wire                                  re_A,
            input wire                                  re_B,
            
            input wire unsigned [PC_BITWIDTH - 1 : 0]   re_addr_A,
            input wire unsigned [PC_BITWIDTH - 1 : 0]   re_addr_B,
            
                                    
        //========== OUTPUT ==========            
		      output logic                                enable_out_A,
            output logic                                enable_out_B,
            
            output logic unsigned [23 : 0]              instruction_out_A,
            output logic unsigned [23 : 0]              instruction_out_B,
            output logic unsigned [31 : 0]              mem_data_out_B
            
);
    
    logic  unsigned [PC_BITWIDTH - 3 : 0]  mem_left_addr_a;
    logic  unsigned [PC_BITWIDTH - 3 : 0]  mem_left_addr_b;
    
    logic  unsigned [PC_BITWIDTH - 3 : 0]  mem_right_addr_a;
    logic  unsigned [PC_BITWIDTH - 3 : 0]  mem_right_addr_b;
            
    wire   unsigned [15 : 0]               mem_left_data_out_a;
    wire   unsigned [15 : 0]               mem_left_data_out_b;
    
    wire   unsigned [15 : 0]               mem_right_data_out_a;
    wire   unsigned [15 : 0]               mem_right_data_out_b;
    
    logic  unsigned [1 : 0]                re_addr_A_last_2_bits;
    logic  unsigned [1 : 0]                re_addr_B_last_2_bits;
    
    logic unsigned [23 : 0]                instruction_out_A_i;
    logic unsigned [23 : 0]                instruction_out_B_i;
    
    
    
    always_ff @(posedge clk, negedge reset_n) begin : last_two_bits_reg_proc
        if (!reset_n) begin
            re_addr_A_last_2_bits <= 0;
            re_addr_B_last_2_bits <= 0;
            
            enable_out_A <= 0;
            enable_out_B <= 0;
            
        end else begin
            re_addr_A_last_2_bits <= re_addr_A [1 : 0];
            re_addr_B_last_2_bits <= re_addr_B [1 : 0];
            
            enable_out_A <= re_A;
            enable_out_B <= re_B;
            
        end
    end : last_two_bits_reg_proc
    
    always_comb begin : mem_addr_proc
        
        if (re_addr_A [1] == 1'b0) begin
            mem_left_addr_a = re_addr_A [PC_BITWIDTH - 1 : 2];
        end else begin
            mem_left_addr_a = re_addr_A [PC_BITWIDTH - 1 : 2] + ($size(mem_left_addr_a))'(1);
        end
        
        mem_right_addr_a = re_addr_A [PC_BITWIDTH - 1 : 2];

        if (we) begin
            mem_left_addr_b = wr_addr;
        end else if (re_addr_B [1] == 1'b0) begin
            mem_left_addr_b = re_addr_B [PC_BITWIDTH - 1 : 2];
        end else begin
            mem_left_addr_b = re_addr_B [PC_BITWIDTH - 1 : 2] + ($size(mem_left_addr_b))'(1);
        end
        
        if (we) begin
            mem_right_addr_b = wr_addr;
        end else begin
            mem_right_addr_b = re_addr_B [PC_BITWIDTH - 1 : 2];
        end
        
    end : mem_addr_proc
    
    
    always_comb begin : output_reorder_proc
        case (re_addr_A_last_2_bits) // synthesis parallel_case 
            2'b00 : begin
                instruction_out_A_i = {mem_left_data_out_a, mem_right_data_out_a [15 : 8]};
            end
            
            2'b01 : begin
                instruction_out_A_i = {mem_left_data_out_a [7 : 0], mem_right_data_out_a};
            end
            
            2'b10 : begin
                instruction_out_A_i = {mem_right_data_out_a, mem_left_data_out_a[15 : 8]};
            end
            
            default : begin
                instruction_out_A_i = {mem_right_data_out_a [7 : 0], mem_left_data_out_a};
            end
            
        endcase
        
        
        case (re_addr_B_last_2_bits) // synthesis parallel_case 
            2'b00 : begin
                instruction_out_B_i = {mem_left_data_out_b, mem_right_data_out_b [15 : 8]};
            end
            
            2'b01 : begin
                instruction_out_B_i = {mem_left_data_out_b [7 : 0], mem_right_data_out_b};
            end
            
            2'b10 : begin
                instruction_out_B_i = {mem_right_data_out_b, mem_left_data_out_b[15 : 8]};
            end
            
            default : begin
                instruction_out_B_i = {mem_right_data_out_b [7 : 0], mem_left_data_out_b};
            end
            
        endcase
        
    end : output_reorder_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : mem_data_out_B_proc
        if (!reset_n) begin
            mem_data_out_B <= 0;
        end else begin
            mem_data_out_B <= {mem_left_data_out_b, mem_right_data_out_b};
        end
        
    end : mem_data_out_B_proc
    
/*  always_ff @(posedge clk, negedge reset_n) begin
        if (!reset_n) begin
            instruction_out_A <= 0;
            instruction_out_B <= 0;
        end else begin
            instruction_out_A <= instruction_out_A_i;
            instruction_out_B <= instruction_out_B_i;
        end
    end
*/
    
    assign instruction_out_A = instruction_out_A_i;
    assign instruction_out_B = instruction_out_B_i;
            
    
    /*
    true_dual_port_ram_single_clock
        #(.DATA_WIDTH (16), .ADDR_WIDTH (PC_BITWIDTH - 2)) mem_left_i (.*,
            .data_a (16'd0), 
            .data_b ({data_in [7 : 0], data_in [15 : 8]}),
            
            .addr_a (mem_left_addr_a), 
            .addr_b (mem_left_addr_b),
            
            .we_a (1'b0), 
            .we_b (we),
            
            .q_a (mem_left_data_out_a), 
            .q_b (mem_left_data_out_b));
    
    true_dual_port_ram_single_clock
        #(.DATA_WIDTH (16), .ADDR_WIDTH (PC_BITWIDTH - 2)) mem_right_i (.*,
            .data_a (16'd0), 
            .data_b ({data_in [23 : 16], data_in [31 : 24]}),
            
            .addr_a (mem_right_addr_a), 
            .addr_b (mem_right_addr_b),
            
            .we_a (1'b0), 
            .we_b (we),
            
            .q_a (mem_right_data_out_a), 
            .q_b (mem_right_data_out_b));

*/

    generate
        
      /*  if (FOR_SIM == SIM_BIST) begin
            

            BIST_dual_port_I_rom_left IROM_left_i (.*,
                    .addr_a (mem_left_addr_a), 
                    .addr_b (mem_left_addr_b),
                    .q_a (mem_left_data_out_a), 
                    .q_b (mem_left_data_out_b));
            
            
            BIST_dual_port_I_rom_right IROM_right_i (.*,
                    .addr_a (mem_right_addr_a), 
                    .addr_b (mem_right_addr_b),
                    .q_a (mem_right_data_out_a), 
                    .q_b (mem_right_data_out_b));
                
        end else if (FOR_SIM == SIM_PRELOAD_CODE) begin
            

            dual_port_I_rom_left IROM_left_i (.*,
                    .addr_a (mem_left_addr_a), 
                    .addr_b (mem_left_addr_b),
                    .q_a (mem_left_data_out_a), 
                    .q_b (mem_left_data_out_b));
            
            
            dual_port_I_rom_right IROM_right_i (.*,
                    .addr_a (mem_right_addr_a), 
                    .addr_b (mem_right_addr_b),
                    .q_a (mem_right_data_out_a), 
                    .q_b (mem_right_data_out_b));
            
        end else begin
      */  
            IRAM_left IRAM_left_i (.*,
                    .we (we),
                    .data_in (data_in [31 : 16]),
                    .addr_a (mem_left_addr_a), 
                    .addr_b (mem_left_addr_b),
                    .q_a (mem_left_data_out_a), 
                    .q_b (mem_left_data_out_b));
            
            
            IRAM_right IRAM_right_i (.*,
                    .we (we),
                    .data_in (data_in [15 : 0]),
                    .addr_a (mem_right_addr_a), 
                    .addr_b (mem_right_addr_b),
                    .q_a (mem_right_data_out_a), 
                    .q_b (mem_right_data_out_b));
            
    //    end
        
    endgenerate
        
endmodule : Instruction_Memory

`default_nettype wire
