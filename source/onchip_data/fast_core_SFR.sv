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
//    SFR (peripherals not included) implementation
//=============================================================================


`include "common.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// fast_core_SFR
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

module fast_core_SFR (
        
        //========== INPUT ==========
        input wire                                      clk,                           
        input wire                                      reset_n,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]        peripheral_data_in,
        
        input wire unsigned [DATA_WIDTH - 1 : 0]        P0_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]        P1_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]        P2_in,
        input wire unsigned [DATA_WIDTH - 1 : 0]        P3_in,
                
        input wire  unsigned [7 : 0]                    read_addr,
        input wire  unsigned [DATA_WIDTH - 1 : 0]       data_in,
        
        input wire  unsigned [DATA_WIDTH - 1 : 0]       data_for_ACC_load,
        
        input wire unsigned [SP_INC_BITS - 1 : 0]       ctl_stack_move_amount,
                
        input wire  unsigned [7 : 0]        write_addr,
        input wire                          we,
        
        input wire                          CY,
        input wire                          AC,
        input wire                          P,
        input wire                          OV,
        
        input wire                          ctl_CY_update,

        input wire unsigned [DATA_WIDTH - 1 : 0]    MOVC_data_in,
        input wire                          ctl_load_MOVC_data,
        
        input wire                          ctl_PSW_flag_update,
        input wire                          ctl_load_A,
        input wire                          ctl_RLC_A,
        input wire                          ctl_RRC_A,
        
        input wire                          ctl_DPTR_update,
        input wire  unsigned [DATA_WIDTH * 2 - 1 : 0]   DPTR_data,
        
        input wire                          ctl_DPTR_inc,
        
        //========== OUTPUT ==========
        output logic unsigned [DATA_WIDTH - 1 : 0]      data_out,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      ACC,
        output logic unsigned [DATA_WIDTH - 1 : 0]      B,
        output logic unsigned [DATA_WIDTH - 1 : 0]      PSW,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P0,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P1,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P2,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P3,
        output logic unsigned [DATA_WIDTH - 1 : 0]      PCON,
        output logic unsigned [DATA_WIDTH - 1 : 0]      DPH,
        output logic unsigned [DATA_WIDTH - 1 : 0]      DPL,
        
        output logic unsigned [DATA_WIDTH - 1 : 0]      P0_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P1_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P2_direction,
        output logic unsigned [DATA_WIDTH - 1 : 0]      P3_direction,
        
        output logic unsigned [DATA_WIDTH * 2 - 1 : 0]  SP,
        output wire  unsigned [DATA_WIDTH - 1 : 0]      XPAGE
);
    
    wire  unsigned [15 : 0]                  DPTR_plus_1;
    wire                                     internal_P;
    logic unsigned [DATA_WIDTH - 1 : 0]      XPAGE_i = 0;
    
    assign DPTR_plus_1 = {DPH, DPL} + 16'd1;
    
    assign internal_P = ^(ACC [7 : 0]);
    
    assign XPAGE = XPAGE_i;
    
    always_ff @(posedge clk, negedge reset_n) begin : memory_read_proc
        if (!reset_n) begin
            data_out <= 0;
        end else begin
            if ((read_addr == write_addr) && we)  begin
                data_out <= data_in;            
            end else begin
                case (read_addr) // synthesis parallel_case 
                    ACC_ADDR : begin
                        if (ctl_load_A) begin
                            data_out <= data_for_ACC_load;
                        end else if (ctl_load_MOVC_data) begin
                            data_out <= MOVC_data_in;
                        end else begin
                            data_out <= ACC;
                        end
                    end
                    
                    B_ADDR : begin
                        data_out <= B;
                    end
                    
                    XPAGE_ADDR: begin
                        data_out <= XPAGE_i;
                    end
                    
                    PSW_ADDR : begin
                        case (1'b1)  // synthesis parallel_case 
                            ctl_PSW_flag_update : begin
                                data_out <= {CY, AC, PSW[5 : 3], OV, PSW[1], P};
                            end
                
                            ctl_RLC_A : begin
                                data_out <= {ACC[7], PSW [6 : 0]};
                            end
                            
                            ctl_RRC_A : begin
                                data_out <= {ACC[0], PSW [6 : 0]};
                            end
                            
                            ctl_CY_update : begin
                                data_out <= {CY, PSW [6 : 0]};
                            end
                            
                            default : begin
                                data_out <= {PSW [7: 1], internal_P};       
                            end
                        endcase
                    end
                                        
                    P0_ADDR : begin
                        data_out <= P0_in;
                    end
                    
                    P1_ADDR : begin
                        data_out <= P1_in;
                    end
                    
                    P2_ADDR : begin
                        data_out <= P2_in;
                    end
                    
                    P3_ADDR : begin
                        data_out <= P3_in;
                    end
                                        
                    PCON : begin
                        data_out <= PCON;
                    end
                    
                    DPL_ADDR : begin
                        
                        if (ctl_DPTR_update) begin
                            data_out <= DPTR_data [7 : 0];
                        end else if (ctl_DPTR_inc) begin
                            data_out <= DPTR_plus_1 [7 : 0];
                        end else begin
                            data_out <= DPL;            
                        end
                        
                    end
                    
                    DPH_ADDR : begin
                        
                        if (ctl_DPTR_update) begin
                            data_out <= DPTR_data [15 : 8];
                        end else if (ctl_DPTR_inc) begin
                            data_out <= DPTR_plus_1 [15 : 8];
                        end else begin
                            data_out <= DPH;
                        end
                        
                         
                        
                    end
                    
                    SP : begin
                        
                        case (1'b1)  
                            (|ctl_stack_move_amount) : begin
                                data_out <= SP [7 : 0] + 
                                    ($size(data_out))'({{(ADDR_WIDTH - SP_INC_BITS){ctl_stack_move_amount[$high(ctl_stack_move_amount)]}}, ctl_stack_move_amount});
                            end     
                            
                            (we & (~(|(write_addr ^ SPL_ADDR)))) : begin
                               //== data_out <= {SP [15 : 8], data_in};
										 data_out <= data_in;
                            end
                            
                            (we & (~(|(write_addr ^ SPH_ADDR)))) : begin
                               //== data_out <=  {data_in, SP [7 : 0]};
										 data_out <=  SP [7 : 0];
                            end
                            
                            default : begin
                                data_out <= SP [7 : 0];
                            end
                        endcase
                        
                            
                    end
                    
                    P0_DIRECTION_ADDR : begin
                        data_out <= P0_direction;
                    end
                    
                    P1_DIRECTION_ADDR : begin
                        data_out <= P1_direction;
                    end
                    
                    P2_DIRECTION_ADDR : begin
                        data_out <= P2_direction;
                    end
                    
                    P3_DIRECTION_ADDR : begin
                        data_out <= P3_direction;
                    end
                                        
                    default : begin
                        data_out <= peripheral_data_in; 
                    end
                    
                endcase
            end
        end 
    end : memory_read_proc
    
    // Accumulator
    always_ff @(posedge clk, negedge reset_n) begin : ACC_proc
        if (!reset_n) begin
            ACC <= 0;
        end else begin
            
            if (ctl_load_A) begin
                ACC <= data_for_ACC_load;
            end else if (ctl_load_MOVC_data) begin
                ACC <= MOVC_data_in;
            end else if ((we & (~(|(write_addr ^ ACC_ADDR)))))  begin
                ACC <= data_in;
            end
        end 
            
    end : ACC_proc
    
    // Register B
    always_ff @(posedge clk, negedge reset_n) begin : B_proc
        if (!reset_n) begin
            B <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ B_ADDR)))) : begin
                    B <= data_in;
                end
            endcase
        end
    end : B_proc
    
    // XPAGE
    always_ff @(posedge clk, negedge reset_n) begin : XPAGE_proc
        if (!reset_n) begin
            XPAGE_i <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ XPAGE_ADDR)))) : begin
                    XPAGE_i <= data_in;
                end
            endcase
        end
    end : XPAGE_proc
    
    
    // PSW (Status)
    always_ff @(posedge clk, negedge reset_n) begin : PSW_proc
        if (!reset_n) begin
            PSW <= 0;
        end else begin
            case (1'b1) // synthesis parallel_case 
				
                (we & (~(|(write_addr ^ PSW_ADDR)))) : begin
                    PSW <= data_in;
                end
                
                ctl_PSW_flag_update : begin
                    PSW <= {CY, AC, PSW[5 : 3], OV, PSW[1], P};
                end
                
                ctl_RLC_A : begin
                    PSW <= {ACC[7], PSW [6 : 1], P};
                end
                
                ctl_RRC_A : begin
                    PSW <= {ACC[0], PSW [6 : 1], P};
                end
                
                ctl_CY_update : begin
                    PSW <= {CY, PSW [6 : 1], P};
                end
					 
					 default : begin
					 
					 end
                
            endcase
        end
    end : PSW_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : P0_proc
        if (!reset_n) begin
            P0 <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ P0_ADDR)))) : begin
                    P0 <= data_in;
                end
            endcase
        end
    end : P0_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : P1_proc
        if (!reset_n) begin
            P1 <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ P1_ADDR)))) : begin
                    P1 <= data_in;
                end
                                
            endcase
        end
    end : P1_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : P2_proc
        if (!reset_n) begin
            P2 <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ P2_ADDR)))) : begin
                    P2 <= data_in;
                end
            endcase
        end
    end : P2_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : P3_proc
        if (!reset_n) begin
            P3 <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ P3_ADDR)))) : begin
                    P3 <= data_in;
                end
                                
            endcase
        end
    end : P3_proc
            
    always_ff @(posedge clk, negedge reset_n) begin : PCON_proc
        if (!reset_n) begin
            PCON <= 0;
        end else begin
            case (1'b1)
                (we & (~(|(write_addr ^ PCON_ADDR)))) : begin
                    PCON <= data_in;
                end
            endcase
        end
    end : PCON_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : DPH_proc
        if (!reset_n) begin
            DPH <= 0;
        end else begin
            if (ctl_DPTR_update) begin
                DPH <= DPTR_data [15 : 8];
            end else if (ctl_DPTR_inc) begin
                DPH <= DPTR_plus_1 [15 : 8];
            end else begin
                case (1'b1)
                    (we & (~(|(write_addr ^ DPH_ADDR)))) : begin
                        DPH <= data_in;
                    end
                endcase
            end
        end
    end : DPH_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : DPL_proc
        if (!reset_n) begin
            DPL <= 0;
        end else begin
            if (ctl_DPTR_update) begin
                DPL <= DPTR_data [7 : 0];
            end else if (ctl_DPTR_inc) begin
                DPL <= DPTR_plus_1 [7 : 0];
            end else begin
                case (1'b1)
                    (we & (~(|(write_addr ^ DPL_ADDR)))) : begin
                        DPL <= data_in;
                    end
                endcase
            end
        end
    end : DPL_proc
    
    always_ff @(posedge clk, negedge reset_n) begin : SP_proc
        if (!reset_n) begin
            SP <= ($size(SP))'(DEFAULT_STACK_START);
        end else begin
            
            case (1'b1) // synthesis parallel_case 
                (|ctl_stack_move_amount) : begin
                    SP <= SP + 
                        {{(ADDR_WIDTH - SP_INC_BITS){ctl_stack_move_amount[$high(ctl_stack_move_amount)]}}, ctl_stack_move_amount};
                end     
                
                (we & (~(|(write_addr ^ SPL_ADDR)))) : begin
                    SP [7 : 0] <= data_in;
                end
                
                (we & (~(|(write_addr ^ SPH_ADDR)))) : begin
                    SP [15 : 8] <= data_in;
                end
					 
					 default : begin
					 
					 end
                
            endcase
        end
    end : SP_proc

    always_ff @(posedge clk, negedge reset_n) begin : port_direction_proc
        if (!reset_n) begin
            P0_direction <= 0;
            P1_direction <= 0;
            P2_direction <= 0;
            P3_direction <= 0;
        end else begin
            if (we & (~(|(write_addr ^ P0_DIRECTION_ADDR)))) begin
                P0_direction <= data_in;
            end
            
            if (we & (~(|(write_addr ^ P1_DIRECTION_ADDR)))) begin
                P1_direction <= data_in;
            end
            
            if (we & (~(|(write_addr ^ P2_DIRECTION_ADDR)))) begin
                P2_direction <= data_in;
            end
            
            if (we & (~(|(write_addr ^ P3_DIRECTION_ADDR)))) begin
                P3_direction <= data_in;
            end
        end 
    end : port_direction_proc

endmodule : fast_core_SFR

`default_nettype wire
