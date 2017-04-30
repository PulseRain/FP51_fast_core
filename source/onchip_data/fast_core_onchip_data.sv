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
//    SFR and RAM (internal and external. Here the "external" is a term used
// by traditional 8051. In fact, for FP51, all RAMs (internal/external) are
// onchip block RAMs
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// fast_core_onchip_data
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      
//
// See Also:
//    
//-----------------------------------------------------------------------------

module fast_core_onchip_data (
      
        //========== INPUT ==========
        
        input wire                      clk,
        input wire                      reset_n,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]        peripheral_data_in,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]    P0_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]    P1_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]    P2_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]    P3_in,
                    
        input wire                      ctl_stack_read,
        input wire                      ctl_stack_write,
        
        input wire unsigned [15 : 0]    read_addr,
        input wire unsigned [15 : 0]    write_addr,
        input wire unsigned [7 : 0]     data_in,
        input wire                      we,
        
        input wire  unsigned [7 : 0]    data_for_ACC_load,
        
        input wire                      wr_indirect1_direct0,
        input wire                      rd_indirect1_direct0,
        
        input wire unsigned [SP_INC_BITS - 1 : 0]   ctl_stack_move_amount,
        
        input wire                      CY,
        input wire                      AC,
        input wire                      P,
        input wire                      OV,
        

        input wire                      ctl_CY_update,
        input wire unsigned [7 : 0]     MOVC_data_in,
        input wire                      ctl_load_MOVC_data,
        
        input wire                      ctl_PSW_flag_update,
        input wire                      ctl_load_A,
        input wire                      ctl_RLC_A,
        input wire                      ctl_RRC_A,
        
        input wire                          ctl_DPTR_update,
        input wire  unsigned [15 : 0]       DPTR_data,
        
        input wire                          ctl_DPTR_inc,
        
        //========== OUTPUT ==========
    
        output wire                        SFR_we,
        output logic unsigned [7 : 0]      data_out,
        
        output wire unsigned [7 : 0]       R0,
        output wire unsigned [7 : 0]       R1,      
        
        
        output wire unsigned [7 : 0]        ACC,
        output wire unsigned [7 : 0]        B,
        output wire unsigned [7 : 0]        PSW,
        output wire unsigned [7 : 0]        P0,
        output wire unsigned [7 : 0]        P1,
        output wire unsigned [7 : 0]        P2,
        output wire unsigned [7 : 0]        P3,
        output wire unsigned [7 : 0]        PCON,
        output wire unsigned [7 : 0]        DPH,
        output wire unsigned [7 : 0]        DPL,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      P0_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P1_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P2_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P3_direction,
        
        output wire unsigned [15 : 0]                   SP,
        output wire unsigned [DATA_WIDTH - 1 : 0]       XPAGE
        
);
    
    wire  unsigned [1 : 0]                  R_bank_active_index;
    wire  unsigned [7 : 0]                  fast_core_RAM_data_out;
    
    wire                                    IRAM_we, XRAM_we, RAM_we;
        
    wire  unsigned [7 : 0]                  fast_core_SFR_data_out;
    
    logic unsigned [15 : 0]                 read_addr_d1, write_addr_d1;
    logic unsigned [7 : 0]                  data_in_d1;
    
    wire                                    is_lower_256_bytes_read;
    wire                                    is_lower_256_bytes_write;
    
    wire                                    is_lower_128_bytes_write;
    
    wire                                    is_SFR_read;
    logic                                   ctl_stack_read_d1;
    logic                                   we_d1;
    
    assign R_bank_active_index = PSW [PSW_RS1_INDEX : PSW_RS0_INDEX];
        
    
    fast_core_reg_bank fast_core_reg_bank_i (.*,
            .active_bank_index (R_bank_active_index),
            //.active_bank_index (2'b00),
            .addr (write_addr),
            .data_in (data_in),
            .we (we),
            .R0 (R0),
            .R1 (R1)
    );
    
    assign is_lower_256_bytes_write = ~(|write_addr [15 : 8]);
    assign is_lower_256_bytes_read = ~(|read_addr_d1 [15 : 8]);
    
    assign is_lower_128_bytes_write = ~(|write_addr [15 : 7]);
    
    assign IRAM_we = we & is_lower_128_bytes_write;
    assign XRAM_we = we & (~is_lower_256_bytes_write);
        
    assign RAM_we = IRAM_we | XRAM_we | ctl_stack_write;
        
    fast_core_RAM fast_core_RAM_i (.*,
            .read_addr (read_addr [15 : 0]),
            .write_addr (write_addr [15 : 0]),
            .data_in (data_in),
            .we (RAM_we),
            .data_out (fast_core_RAM_data_out));
    
    always_ff @(posedge clk, negedge reset_n) begin
        if (!reset_n) begin
            read_addr_d1 <= 0;
            write_addr_d1 <= 0;
            data_in_d1 <= 0;
            ctl_stack_read_d1 <= 0;
            we_d1 <= 0;
        end else begin
            read_addr_d1 <= read_addr;
            write_addr_d1 <= write_addr;
            data_in_d1 <= data_in;
            ctl_stack_read_d1 <= ctl_stack_read;
            we_d1 <= we;
        end
    end
    
    assign SFR_we = we & is_lower_256_bytes_write & (~ctl_stack_write) & (write_addr[7]);
        
    fast_core_SFR fast_core_SFR_i (.*,
            
            .peripheral_data_in (peripheral_data_in),
            
            .P0_in (P0_in),
            .P1_in (P1_in),
            .P2_in (P2_in),
            .P3_in (P3_in),
            
            .P0_direction (P0_direction),
            .P1_direction (P1_direction),
            .P2_direction (P2_direction),
            .P3_direction (P3_direction),
                        
            .read_addr (read_addr [7 : 0]),
            .write_addr (write_addr [7 : 0]),
            .data_in (data_in),
            
            .we (SFR_we),
            
            .MOVC_data_in (MOVC_data_in),
            .ctl_load_MOVC_data (ctl_load_MOVC_data),
        
            .ctl_DPTR_update (ctl_DPTR_update),
            .DPTR_data (DPTR_data),
            .ctl_DPTR_inc (ctl_DPTR_inc),
            
            .ctl_stack_move_amount (ctl_stack_move_amount),
            
            .data_out (fast_core_SFR_data_out)
            
            
    );

    assign is_SFR_read = read_addr_d1[7] & is_lower_256_bytes_read & (~ctl_stack_read_d1);
    
    always_comb begin
        
        if ( (read_addr_d1 != write_addr_d1) || (!we_d1) ) begin 
            data_out = is_SFR_read ? fast_core_SFR_data_out : fast_core_RAM_data_out;
        end else begin
            data_out = data_in_d1;
        end
    end
 
endmodule : fast_core_onchip_data

`default_nettype wire
