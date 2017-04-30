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
//    control (instruction decoder) for FP51 fast core. 
//=============================================================================

`include "common.svh"
`include "MCS_51_Instructions.svh"
`include "SFR.svh"
`include "fast_core_common.svh"

`default_nettype none

//-----------------------------------------------------------------------------
// fast_core_control
//
// Input:
//     see port list
//
// Output:
//     see port list
//
// Remarks:
//      translate instructions into control signals
//
// See Also:
//    
//-----------------------------------------------------------------------------

module fast_core_control (
        
        //========== INPUT ==========
        
        input   wire                            clk,
        input   wire                            reset_n,
        
        input   wire                            Prog_Mem_enable_in,
        input   wire unsigned [23 : 0]          Prog_Mem_data_in_A,
        input   wire unsigned [23 : 0]          Prog_Mem_data_in_B,
        
        input   wire                            pipeline_stall,
        input   wire                            reg_jump_active,
        
        input   wire                            int_gen_flag,
        input   wire unsigned [7 : 0]           int_addr_reg,
                
        input   wire  unsigned [PC_BITWIDTH - 1 : 0]    next_PC,        
        
        input   wire unsigned [15 : 0]          DPTR,
        input   wire unsigned [7 : 0]           R0,
        input   wire unsigned [7 : 0]           R1,
        input   wire unsigned [1 : 0]           active_reg_bank_index,
        
        input   wire                            ctl_pipeline_hold,
        input   wire                            ctl_pipeline_resume,    
        
        input   wire                            post_control_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   post_control_IR,
        
        input   wire                            preALU_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   preALU_IR,
        input   wire                            ALU_indirect_R0_R1_access_flag,
        input   wire [23 : 0]                   ALU_IR,
    
        //========== OUTPUT ==========
        output logic                            ctl_fetch_instruction_request,
        output logic                            ctl_ALU_adjust_a,
        output logic                            ctl_ALU_adjust_b,
        output logic                            ctl_ALU_data_a,
        output logic                            ctl_ALU_data_b,
        output logic                            ctl_PSW_flag_update,
        output logic                            ctl_load_A,
                
        output logic unsigned [7 : 0]           ALU_adjust_a,
        output logic unsigned [7 : 0]           ALU_adjust_b,
        
        output logic unsigned [7 : 0]           data_rd_address,
        output logic unsigned [7 : 0]           data_wr_address,
        
        output logic unsigned [15 : 0]          xdata_rd_address,
        output logic unsigned [15 : 0]          xdata_wr_address,
        
        output logic                            ctl_indirect_read,
        output logic                            ctl_indirect_write,
        
        
        output logic [ALU_NUM_OF_OPERATIONS - 1 : 0]    ctl_ALU_cmd,
        output logic [PC_NUM_OF_OPERATIONS - 1:0]       ctl_PC_cmd,
        output logic unsigned [23 : 0]                  IR,
        output logic unsigned [PC_BITWIDTH - 1 : 0]     PC,
        output logic unsigned [1 : 0]                   prefetch_size_adjust,
        output wire  unsigned [1 : 0]                   size_of_instruction,
            
        output logic                                    ctl_ALU_active,
        output logic                                    ctl_write_enable,
        output logic                                    ctl_xread_enable,
        output logic                                    ctl_xwrite_enable,
        
        output logic                                    ctl_DPTR_load_data,
        output logic                                    ctl_DPTR_inc,
        output logic                                    ctl_DPTR_read,
        output logic                                    ctl_DPTR_write,
                
        output logic unsigned [SP_INC_BITS - 1 : 0]     ctl_stack_move_amount,
        output logic                                    ctl_stack_read,
        output logic                                    ctl_stack_write,
        output logic                                    ctl_reset_I_fetch,
        output logic                                    ctl_interruptable
        
);
    
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Signals
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
    
        //---------------------------------------------------------------------
        // prefetch buffer
        //---------------------------------------------------------------------
            enum {OP_PREFETCH_A_3_BYTE, OP_PREFETCH_SEQUENTIAL} op_prefetch = 0;
            localparam PREFETCH_NUM_OF_OPERATIONS = op_prefetch.num();
            logic [PREFETCH_NUM_OF_OPERATIONS - 1:0] ctl_prefetch_cmd;
            
            logic unsigned [23 : 0]             prefetch_buffer;
            logic unsigned [23 : 0]             prefetch_buffer_tmp;
            
            wire  unsigned [1 : 0]              size_of_instruction_1st_after_IR;
            wire  unsigned [1 : 0]              size_of_instruction_2nd_after_IR;
            wire  unsigned [1 : 0]              size_of_instruction_3rd_after_IR;
            
            logic unsigned [1 : 0]              prefetch_size_adjust_d1;
            logic                               ctl_fetch_instruction_request_d1;
            
        //---------------------------------------------------------------------
        // Instruction Register
        //---------------------------------------------------------------------
            enum {OP_IR_LOAD_FROM_B, OP_IR_LOAD_SEQUENTIAL} op_IR = 0;
            localparam IR_NUM_OF_OPERATIONS = op_IR.num();
            logic [IR_NUM_OF_OPERATIONS - 1:0]  ctl_IR_cmd;
            
            wire  unsigned [7 : 0]              indirect_R;
            wire  unsigned [7 : 0]              direct_addr;
            wire  unsigned [7 : 0]              active_R;
            
            wire  unsigned [15 : 0]             active_Reg_address;
        //---------------------------------------------------------------------
        // control command out
        //---------------------------------------------------------------------
            
            wire                                run_ext_flag;
            wire                                run_ext_ext_flag;
            logic                               ctl_run_ext;
            logic                               ctl_run_ext_ext;
            
            logic  unsigned [2 : 0]             wait_counter;
            logic                               ctl_load_div_wait_counter;
            logic                               ctl_da_counter;
            logic                               ctl_movc_counter;
            logic                               ctl_fill_pipe;
            logic                               ctl_reset_size_of_instruction_d1;
            
            // op_ALU cast for debug, one-hot translation, 
            // enum value can be shown in the simulation in this way
            // Hopefully, synthesizer will optimize out the "op_ALU" variable
            
            // synthesis translate_off
            ///////////////////////////////////////////////////////////////////////
                op_ALU_t op_ALU_debug;
                always_comb begin : op_ALU_cast_for_debug
                    for (int i = 0; i < ALU_NUM_OF_OPERATIONS; ++i) begin
                        if (ctl_ALU_cmd[i]) begin
                            $cast(op_ALU_debug, i);
                        end
                    end
                end : op_ALU_cast_for_debug
            ///////////////////////////////////////////////////////////////////////
            // synthesis translate_on   

    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // PC
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        // op_PC cast for debug, one-hot translation, 
        // enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "op_PC" variable
        
        // synthesis translate_off
        
        ///////////////////////////////////////////////////////////////////////
            op_PC_t op_PC_debug;
            always_comb begin : op_PC_cast_for_debug
                for (int i = 0; i < PC_NUM_OF_OPERATIONS; ++i) begin
                    if (ctl_PC_cmd[i]) begin
                        $cast(op_PC_debug, i);
                    end
                end
            end : op_PC_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on   
                    
        
        always_ff @(posedge clk, negedge reset_n) begin : PC_proc
            if (!reset_n) begin
                PC <= 0;
            end else if (int_gen_flag) begin
                PC <= {8'd0, int_addr_reg};
            end else if (reg_jump_active) begin
                PC <= next_PC;
            end else begin
                case (1'b1)
                    ctl_PC_cmd[OP_PC_SEQUENTIAL] : begin
                        PC <= PC + size_of_instruction;
                    end
                endcase
            end
        end : PC_proc           
                
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // prefetch buffer
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
        // op_prefetch cast for debug, one-hot translation, 
        // enum value can be shown in the simulation in this way
        // Hopefully, synthesizer will optimize out the "op_prefetch" variable
        
        // synthesis translate_off
        ///////////////////////////////////////////////////////////////////////
            always_comb begin : op_prefetch_cast_for_debug
                for (int i = 0; i < PREFETCH_NUM_OF_OPERATIONS; ++i) begin
                    if (ctl_prefetch_cmd[i]) begin
                        $cast(op_prefetch, i);
                    end
                end
            end : op_prefetch_cast_for_debug
        ///////////////////////////////////////////////////////////////////////
        // synthesis translate_on
        
        always_ff @(posedge clk, negedge reset_n) begin : prefetch_buffer_proc
            if (!reset_n) begin
                prefetch_buffer <= 0;
            end else if (reg_jump_active | int_gen_flag) begin
                prefetch_buffer <= 0;
            end else if ((~ctl_fetch_instruction_request_d1) & ctl_prefetch_cmd[OP_PREFETCH_SEQUENTIAL]) begin
                if (size_of_instruction == 2'b10) begin
                    prefetch_buffer <= {prefetch_buffer [7 : 0], prefetch_buffer_tmp [23 : 8]};
                end else if (size_of_instruction == 2'b01) begin
                    prefetch_buffer <= {prefetch_buffer [15 : 0], prefetch_buffer_tmp [23 : 16]};
                end else begin
                    prefetch_buffer <= prefetch_buffer_tmp;
                end
            end else begin
                case(1'b1)
                    ctl_prefetch_cmd[OP_PREFETCH_A_3_BYTE] : begin
                        prefetch_buffer <= Prog_Mem_data_in_A;
                    end
                    
                    ctl_prefetch_cmd[OP_PREFETCH_SEQUENTIAL] : begin
                        
                        if (size_of_instruction == 2'b10) begin
                            prefetch_buffer <= {prefetch_buffer [7 : 0], Prog_Mem_data_in_A [23 : 8]};
                        end else if (size_of_instruction == 2'b01) begin
                            prefetch_buffer <= {prefetch_buffer [15 : 0], Prog_Mem_data_in_A [23 : 16]};
                        end else begin
                            prefetch_buffer <= Prog_Mem_data_in_A;
                        end
                        
                    end
                    
                    default : begin
                    end
                                            
                endcase
                
            end
            
            
        end : prefetch_buffer_proc  
        
        
        always_ff @(posedge clk, negedge reset_n) begin : prefetch_buffer_tmp_proc
            if (!reset_n) begin
                prefetch_buffer_tmp <= 0;
            end else if ((~ctl_fetch_instruction_request) & ctl_fetch_instruction_request_d1) begin
                prefetch_buffer_tmp <= Prog_Mem_data_in_A;
            end
        end : prefetch_buffer_tmp_proc
            
        
        always_ff @(posedge clk, negedge reset_n) begin : pipeline_stall_delay_proc
            if (!reset_n) begin
                ctl_fetch_instruction_request_d1 <= 0;
            end else begin
                ctl_fetch_instruction_request_d1 <= ctl_fetch_instruction_request;
            end
        end : pipeline_stall_delay_proc
        
        always_ff @(posedge clk, negedge reset_n) begin
            if (!reset_n) begin
                prefetch_size_adjust_d1 <= 0;
            end else if (ctl_reset_size_of_instruction_d1) begin
                prefetch_size_adjust_d1 <= 0;
            end else if (ctl_fetch_instruction_request) begin
                prefetch_size_adjust_d1 <= prefetch_size_adjust;
            end
        end
            
        Instruction_Size inst_size_i (.*,
            .instruction_byte (IR [23 : 16]),
            .size_of_instruction (size_of_instruction) );
        
        Instruction_Size inst_size_1st_after_IR (.*,
            .instruction_byte (IR [15 : 8]),
            .size_of_instruction (size_of_instruction_1st_after_IR) );
        
        Instruction_Size inst_size_2nd_after_IR (.*,
            .instruction_byte (IR [7 : 0]),
            .size_of_instruction (size_of_instruction_2nd_after_IR) );
        
        Instruction_Size inst_size_3rd_after_IR (.*,
            .instruction_byte (prefetch_buffer [23 : 16]),
            .size_of_instruction (size_of_instruction_3rd_after_IR) );
        
        always_comb begin
            if (ctl_fill_pipe) begin
                prefetch_size_adjust = (size_of_instruction);
            end else begin
                case (prefetch_size_adjust_d1) // synthesis parallel_case 
                    2'b01 : begin
                        prefetch_size_adjust = (size_of_instruction_1st_after_IR);
                    end
                    
                    2'b10 : begin
                        prefetch_size_adjust = (size_of_instruction_2nd_after_IR);
                    end
                    
                    3'b11 : begin
                        prefetch_size_adjust = (size_of_instruction_3rd_after_IR);
                    end
                    
                    default : begin
                       prefetch_size_adjust = 2'b11;    
                    end
                endcase
            end
            
        end
        
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Instruction Register
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                        
            // op_IR cast for debug, one-hot translation, 
            // enum value can be shown in the simulation in this way
            // Hopefully, synthesizer will optimize out the "op_IR" variable
            
            // synthesis translate_off
            ///////////////////////////////////////////////////////////////////////
                always_comb begin : op_IR_cast_for_debug
                    for (int i = 0; i < IR_NUM_OF_OPERATIONS; ++i) begin
                        if (ctl_IR_cmd[i]) begin
                            $cast(op_IR, i);
                        end
                    end
                end : op_IR_cast_for_debug
            ///////////////////////////////////////////////////////////////////////
            // synthesis translate_on   
            
            // Instruction Register
            always_ff @(posedge clk, negedge reset_n) begin : IR_proc
                if (!reset_n) begin
                    IR <= 0;
                end else if (reg_jump_active | int_gen_flag) begin
                    IR <= 0;
                end else begin
                    case (1'b1) // synthesis parallel_case 
						  
                        ctl_IR_cmd [OP_IR_LOAD_FROM_B]: begin
                            IR <= Prog_Mem_data_in_B;
                        end
                        
                        ctl_IR_cmd [OP_IR_LOAD_SEQUENTIAL]: begin
                            if (size_of_instruction == 2'b10) begin
                                IR <= {IR[7 : 0], prefetch_buffer [23 : 8]};
                            end else if (size_of_instruction == 2'b01) begin
                                IR <= {IR[15 : 0], prefetch_buffer [23 : 16]};
                            end else begin
                                IR <= prefetch_buffer;
                            end
                        end
								
								default : begin
								
								end
								
                    endcase
                end
            end : IR_proc
            
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // Div counter
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
            
            always_ff @(posedge clk, negedge reset_n) begin
                if (!reset_n) begin
                    wait_counter <= 0;
                end else if (ctl_load_div_wait_counter) begin
                    wait_counter <= 6;
                end else if (ctl_movc_counter) begin
                    wait_counter <= 3;
                end else if (ctl_da_counter) begin
                    wait_counter <= 2;
                end else if (wait_counter && (!pipeline_stall)) begin
                    wait_counter <= wait_counter - ($size(wait_counter))'(1);
                end
            end
            
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // FSM
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                
        enum {S_INIT, S_FILL_PIPE_1ST, S_FILL_PIPE_2ND, S_FILL_PIPE_3RD,  
              S_RUN_NORMAL, S_RUN_EXT, S_RUN_EXT_EXT} states = 0;
                
        localparam FSM_NUM_OF_STATES = states.num();
        logic [FSM_NUM_OF_STATES - 1:0] current_state = 0, next_state;
                
        assign indirect_R  = (IR[16]) ? R1 : R0;
        assign direct_addr = IR [15 : 8];
        assign active_R = {3'd0, active_reg_bank_index, IR [18 : 16]};
        
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
            
                    
        assign run_ext_flag = ctl_run_ext;
        assign run_ext_ext_flag = ctl_run_ext_ext;
                    
        assign active_Reg_address = {11'd0, active_reg_bank_index, 2'b00, IR [16]};
        
        // FSM main body
        always_comb begin : state_machine_comb

            next_state = 0;
            
            data_rd_address = active_R;
            data_wr_address = ACC_ADDR;
                        
            xdata_rd_address = DPTR;
            xdata_wr_address = DPTR;
            
            ctl_xwrite_enable = 0;
            ctl_xread_enable = 0;
            
            ctl_indirect_read = 0;
            ctl_indirect_write = 0;
            
            ALU_adjust_a = 0;
            ALU_adjust_b = 0;
            
            ctl_fetch_instruction_request = 0;
            
            ctl_IR_cmd = 0;
            ctl_prefetch_cmd = 0;
            
            ctl_ALU_cmd = 0;
            ctl_PC_cmd = 0;
        
            ctl_ALU_adjust_a = 0;
            
            ctl_ALU_adjust_b = 0;
            ctl_ALU_data_a   = 0;
            ctl_ALU_data_b   = 0;
        
            ctl_PSW_flag_update = 0;
            ctl_load_A = 0;
        
            ctl_indirect_read = 0;
            ctl_indirect_write = 0;
                        
            ctl_ALU_active = 0;
            
                
            ctl_write_enable = 1;
            
            ctl_run_ext = 0;
            
            ctl_run_ext_ext = 0;
            
            ctl_load_div_wait_counter = 0;
            
            ctl_da_counter = 0;
            
            ctl_movc_counter = 0;
            
            ctl_DPTR_load_data = 0;
            ctl_DPTR_inc = 0;
            
            ctl_DPTR_read = 0;
            ctl_DPTR_write = 0;             
        
            ctl_stack_move_amount = 0;
            ctl_stack_read = 0;
            ctl_stack_write = 0;
            
            ctl_reset_I_fetch = 0;
            
            ctl_interruptable = 1'b1;
            
            ctl_fill_pipe = 0;
            
            ctl_reset_size_of_instruction_d1 = 0;
            
            if (int_gen_flag) begin
                next_state [S_INIT] = 1;
                ctl_reset_I_fetch = 1'b1;
            end else begin
                case (1'b1) // synthesis parallel_case 
                    
                    current_state[S_INIT]: begin
                        next_state [S_FILL_PIPE_1ST] = 1;
                        ctl_fetch_instruction_request = 1;
                    end                 
                    
                    current_state [S_FILL_PIPE_1ST] : begin
                        ctl_reset_size_of_instruction_d1 = 1'b1;
                        next_state [S_FILL_PIPE_2ND] = 1;
                    end
                                        
                    current_state [S_FILL_PIPE_2ND] : begin
                        ctl_fetch_instruction_request = 1;
                        ctl_IR_cmd[OP_IR_LOAD_FROM_B] = 1;
                        ctl_prefetch_cmd[OP_PREFETCH_A_3_BYTE] = 1;
                        next_state [S_FILL_PIPE_3RD] = 1'b1;
                    end
                                
                    current_state [S_FILL_PIPE_3RD] : begin
                        ctl_fill_pipe = 1'b1;
                        ctl_fetch_instruction_request = 1;
                        next_state [S_RUN_NORMAL] = 1'b1;
                    end
                                        
                    current_state[S_RUN_NORMAL]: begin
                        
                                            
                        if (reg_jump_active) begin
                                next_state [S_INIT] = 1;
                                ctl_reset_I_fetch = 1'b1;
                        end else if (!pipeline_stall) begin
                            if (run_ext_flag) begin
                                ctl_ALU_active = 1; 
                                next_state [S_RUN_EXT] = 1;
                            end else if (run_ext_ext_flag) begin
                                ctl_ALU_active = 1; 
                                next_state [S_RUN_EXT_EXT] = 1;
                            end else begin
                                ctl_ALU_active = 1; 
                                ctl_fetch_instruction_request = 1;
                                
                                ctl_PC_cmd[OP_PC_SEQUENTIAL] = 1;
                                ctl_prefetch_cmd[OP_PREFETCH_SEQUENTIAL] = 1;
                                ctl_IR_cmd [OP_IR_LOAD_SEQUENTIAL] = 1;
                                
                                next_state [S_RUN_NORMAL] = 1;
                            end
                            
                        end else begin
                            next_state [S_RUN_NORMAL] = 1;
                        end
                    
                        
                        casex (IR[23 : 16])  // synthesis parallel_case 
                            // ADD A, Rn                        
                            {INST_ADD_A_R_MSB, 3'b???} : begin
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                                                    
                            INST_ADD_A_DIR : begin
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                ctl_PSW_flag_update = 1;
                                data_rd_address = direct_addr;
                            end
                            
                            INST_ADD_A_DATA : begin
                                ctl_ALU_data_b = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_ADD_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                ctl_PSW_flag_update = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                            
                            end
                                                    
                            {INST_ADDC_A_R_MSB, 3'b???} : begin
                                ctl_ALU_cmd [OP_ALU_ADDC] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            INST_ADDC_A_DIR : begin
                                ctl_ALU_cmd [OP_ALU_ADDC] = 1;
                                data_rd_address = direct_addr;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            INST_ADDC_A_DATA : begin
                                ctl_ALU_data_b = 1;
                                ctl_ALU_cmd [OP_ALU_ADDC] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_ADDC_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_ADDC] = 1;
                                ctl_PSW_flag_update = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                            
                            end
                            
                            {INST_SUBB_A_R_MSB, 3'b???} : begin
                                ctl_ALU_cmd [OP_ALU_SUBC] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            INST_SUBB_A_DIR : begin
                                ctl_ALU_cmd [OP_ALU_SUBC] = 1;
                                data_rd_address = direct_addr;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_SUBB_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_SUBC] = 1;
                                ctl_PSW_flag_update = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                            end
                            
                            INST_SUBB_A_DATA : begin
                                ctl_ALU_data_b = 1;
                                ctl_ALU_cmd [OP_ALU_SUBC] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            INST_INC_A : begin
                                ALU_adjust_b = 8'h1;
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                            end
                        
                            {INST_INC_R_MSB, 3'b???} : begin
                                ALU_adjust_a = 8'h1;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_wr_address = active_R;
                            end
                            
                            INST_INC_DIR : begin
                                ALU_adjust_a = 8'h1;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                                    
                            {INST_INC_AT_R_MSB, 1'b?} : begin
                                ALU_adjust_a = 8'h1;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_rd_address = indirect_R;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_read = 1;
                                ctl_indirect_write = 1;
                            end
                            
                            INST_DEC_A : begin
                                ALU_adjust_b = 8'hFF;
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_rd_address = ACC_ADDR;
                            end
                            
                            {INST_DEC_R_MSB, 3'b???} : begin
                                ALU_adjust_a = 8'hFF;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_wr_address = active_R;
                            end
                            
                            INST_DEC_DIR : begin
                                ALU_adjust_a = 8'hFF;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            {INST_DEC_AT_R_MSB, 1'b?} : begin
                                ALU_adjust_a = 8'hFF;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_rd_address = indirect_R;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_read = 1;
                                ctl_indirect_write = 1;
                            end
                            
                            INST_INC_DPTR : begin
                                ctl_DPTR_inc = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_MUL_AB : begin
                                ctl_run_ext_ext = 1;
                                
                                ctl_ALU_cmd [OP_ALU_MUL] = 1;
                                ctl_ALU_adjust_b = 1;
                                                                
                            end
                            
                            INST_DIV_AB : begin
                                ctl_run_ext_ext = 1;
                                ctl_load_div_wait_counter = 1'b1;
                                
                                ctl_ALU_cmd [OP_ALU_DIV_START] = 1;
                                ctl_ALU_adjust_b = 1;
                                
                                ctl_interruptable = 0;
                            end
                            
                            INST_DA_A : begin
                                ctl_run_ext_ext = 1'b1;
                                ctl_ALU_cmd [OP_ALU_DA_START] = 1'b1;
                                ctl_ALU_adjust_b = 1'b1;
                                ctl_da_counter = 1'b1;
                                ctl_interruptable = 0;
                            end
                                                    
                            {INST_ANL_A_R_MSB, 3'b???} : begin
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                            end
                        
                            INST_ANL_A_DIR : begin
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                                data_rd_address = direct_addr;
                            end
                            
                            {INST_ANL_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                            end
                            
                            INST_ANL_A_DATA : begin
                                ctl_ALU_data_b = 1;
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                            end
                            
                            INST_ANL_DIR_A : begin
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_ANL_DIR_DATA : begin
                                ctl_ALU_data_a = 1;
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                        
                            {INST_ORL_A_R_MSB, 3'b???} : begin
                                ctl_ALU_cmd [OP_ALU_OR] = 1;
                            end
                            
                            INST_ORL_A_DIR : begin
                                ctl_ALU_cmd [OP_ALU_OR] = 1;
                                data_rd_address = direct_addr;
                            end
                            
                            {INST_ORL_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_OR] = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                            end
                                                    
                            INST_ORL_A_DATA : begin
                                ctl_ALU_data_b = 1;
                                ctl_ALU_cmd [OP_ALU_OR] = 1;
                            end
                            
                            INST_ORL_DIR_A : begin
                                ctl_ALU_cmd [OP_ALU_OR] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_ORL_DIR_DATA : begin
                                ctl_ALU_data_a = 1;
                                ctl_ALU_cmd [OP_ALU_OR] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                        
                            {INST_XRL_A_R_MSB, 3'b???} : begin
    
                            end
                            
                            INST_XRL_A_DIR : begin
                                data_rd_address = direct_addr;
                            end
                            
                            {INST_XRL_A_AT_R_MSB, 1'b?} : begin
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                            end
                            
                            INST_XRL_A_DATA : begin
                                ctl_ALU_data_b = 1;
                            end
                            
                            INST_XRL_DIR_A : begin
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_XRL_DIR_DATA : begin
                                ctl_ALU_data_a = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_CLR_A : begin
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_AND] = 1;
                            end
                            
                            INST_CPL_A : begin
                                ctl_ALU_adjust_b = 1;
                                ALU_adjust_b = 8'hFF;
                            end
                                
                                
                            INST_SWAP_A : begin
                                ctl_ALU_cmd[OP_ALU_SWAP_A] = 1;
                                ctl_run_ext_ext = 1'b1;
                                ctl_ALU_adjust_b = 1'b1;
                            end
                            
                            {INST_MOV_A_R_MSB, 3'b???} : begin
                                ctl_load_A = 1;
                                ctl_write_enable = 0;
                            end
                                                                            
                            INST_MOV_A_DIR : begin
                                data_rd_address = direct_addr;
                                ctl_load_A = 1;
                                ctl_write_enable = 0;
                            end
                            
                            {INST_MOV_A_AT_R_MSB, 1'b?} : begin
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                                ctl_load_A = 1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_MOV_A_DATA : begin
                                ctl_ALU_data_b = 1;
                                ctl_load_A = 1;
                                ctl_write_enable = 0;
                            end
                                                                            
                            {INST_MOV_R_A_MSB, 3'b???} : begin
                                ctl_ALU_adjust_b = 1;
                                data_wr_address = active_R;
                            end
                            
                            {INST_MOV_R_DIR_MSB, 3'b???} : begin
                                ctl_ALU_adjust_a = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = active_R;
                            end
                            
                            {INST_MOV_R_DATA_MSB, 3'b???} : begin
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_data_b = 1;
                                data_wr_address = active_R;
                            end
                            
                            INST_MOV_DPTR_DATA : begin
                                ctl_ALU_cmd[OP_ALU_DPTR_DATA] = 1'b1;
                                ctl_ALU_adjust_b = 1'b1;
                                ctl_DPTR_load_data = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_MOV_DIR_A : begin
                                ctl_ALU_adjust_b = 1;
                                data_wr_address = direct_addr;
                            end
                            
                            {INST_MOV_DIR_R_MSB, 3'b???} : begin
                                ctl_ALU_adjust_a = 1;
                                data_wr_address = direct_addr;
                            end
                            
                            {INST_MOV_DIR_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_adjust_a = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_MOV_DIR_DATA : begin
                                ctl_ALU_data_a = 1;
                                ctl_ALU_adjust_b = 1;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_MOV_DIR_DIR : begin
                                ctl_ALU_adjust_a = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = IR [7 : 0];
                            end
                            
                            {INST_MOV_AT_R_A_MSB, 1'b?} : begin
                                ctl_ALU_adjust_b = 1;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_write = 1;
                            end
                            
                            {INST_MOV_AT_R_DIR_MSB, 1'b?} : begin
                                ctl_ALU_adjust_a = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_write = 1;
                            end
                            
                            {INST_MOV_AT_R_DATA_MSB, 1'b?} : begin
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_data_b = 1;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_write = 1;
                            end
                            
                            INST_MOVC_A_DPTR : begin
                                ctl_run_ext_ext = 1'b1;
                                ctl_DPTR_read   = 1'b1;
                                ctl_ALU_cmd[OP_ALU_MOVC_A_DPTR] = 1'b1;
                                ctl_ALU_adjust_b = 1'b1;
                                ctl_movc_counter = 1'b1;
                                ctl_interruptable = 0;
                            end
                            
                            INST_MOVC_A_PC : begin
                                ctl_run_ext_ext = 1'b1;
                                ctl_ALU_cmd[OP_ALU_MOVC_A_PC] = 1'b1;
                                ctl_ALU_adjust_b = 1'b1;
                                ctl_movc_counter = 1'b1;
                                ctl_interruptable = 0;
                            end
                            
                            {INST_MOVX_A_AT_R_MSB, 1'b?} : begin
                                ctl_xread_enable = 1'b1;
                                xdata_rd_address = {8'd0, indirect_R};
                                ctl_ALU_adjust_a = 1;
                                ctl_indirect_read = 1;
                            end
                            
                            INST_MOVX_A_AT_DPTR : begin
                                ctl_DPTR_read    = 1'b1;
                                ctl_ALU_adjust_a = 1'b1;
                            end
                            
                            {INST_MOVX_AT_R_A_MSB, 1'b?} : begin
                                xdata_wr_address = active_Reg_address;
                                ctl_ALU_adjust_b = 1'b1;
                                ctl_xwrite_enable = 1;
                                ctl_indirect_write = 1;
                            end
                            
                            INST_MOVX_AT_DPTR_A : begin
                                ctl_ALU_adjust_b = 1'b1;
                                ctl_DPTR_write   = 1'b1;
                            end
                        
                            // XCH A, Rn = MOV A, Rn + MOV Rn, A
                            {INST_XCH_A_R_MSB, 3'b???} : begin
                                ctl_ALU_cmd [OP_ALU_A_DIRECT_PASS] = 1;
                                ctl_load_A = 1;
                                data_wr_address = active_R;
                            end
                            
                            // XCH A, direct = MOVE A, direct + MOV direct, A
                            INST_XCH_A_DIR : begin
                                ctl_ALU_cmd [OP_ALU_A_DIRECT_PASS] = 1;
                                ctl_load_A = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            // XCH A, @Ri = MOV A, @Ri + MOV @Ri, A
                            {INST_XCH_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_A_DIRECT_PASS] = 1;
                                ctl_load_A = 1;
                                data_rd_address = indirect_R;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_read = 1;
                                ctl_indirect_write = 1;
                            end
                            
                            // XCHD A, @Ri
                            {INST_XCHD_A_AT_R_MSB, 1'b?} : begin
                                ctl_run_ext_ext = 1;
                                ctl_ALU_cmd [OP_ALU_XCHD] = 1;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_PUSH : begin
                                ctl_ALU_adjust_a = 1;
                                data_rd_address = direct_addr;
                                
                                ctl_stack_move_amount = 3'b001;
                                ctl_stack_write = 1'b1;
                                                
                            end
                            
                            INST_POP : begin
                                ctl_ALU_adjust_a = 1;
                                data_wr_address = direct_addr;
                                
                                ctl_stack_move_amount = 3'b111;
                                ctl_stack_read = 1'b1;
                            end
                            
                            INST_JC : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_JNC : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_JB : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_JNB : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_JBC : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                                ctl_interruptable = 0; // because we need to see PC == reg_PC in next stage
                            end     
                            
                            INST_JZ : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_JNZ : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_CJNE_A_DIR_REL : begin
                                ctl_run_ext = 1'b1;
                                data_rd_address = direct_addr;
                                ctl_write_enable = 0;
                                ctl_ALU_cmd [OP_ALU_SUB] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            INST_CJNE_A_DATA_REL : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                                ctl_ALU_data_b = 1;
                                ctl_ALU_cmd [OP_ALU_SUB] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_CJNE_R_DATA_REL_MSB, 3'b???} : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                                ctl_ALU_adjust_a = 1;
                                ALU_adjust_a = IR [15 : 8];
                                ctl_ALU_cmd [OP_ALU_SUB_NEG] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_CJNE_AT_R_DATA_REL_MSB, 1'b?} : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                                data_rd_address = indirect_R;
                                ctl_indirect_read = 1;
                                
                                ctl_ALU_adjust_a = 1;
                                ALU_adjust_a = IR [15 : 8];
                                ctl_ALU_cmd [OP_ALU_SUB_NEG] = 1;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_DJNZ_R_REL_MSB, 3'b???} : begin
                                ctl_run_ext = 1'b1;
                                ctl_interruptable = 0;
                                
                                ALU_adjust_a = 8'hFF;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_wr_address = active_R;
                            end
                            
                            INST_DJNZ_DIR_REL : begin
                                ctl_run_ext = 1'b1;
                                ctl_interruptable = 0;
                                
                                ALU_adjust_a = 8'hFF;
                                ctl_ALU_adjust_a = 1;
                                ctl_ALU_cmd [OP_ALU_ADD] = 1;
                                data_rd_address = direct_addr;
                                data_wr_address = direct_addr;
                            end
                            
                            INST_SJMP : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            {3'b???, INST_AJMP_LSB} : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            {3'b???, INST_ACALL_LSB} : begin
                                ctl_run_ext = 1'b1;
                                ctl_interruptable = 0;
                                ctl_stack_move_amount = 3'b001;
                                ctl_stack_write = 1'b1;
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_2_LOW] = 1;
            
                            end
                                    
                            INST_LJMP : begin
                                ctl_run_ext = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_LCALL : begin
                                ctl_run_ext = 1'b1;
                                ctl_interruptable = 0;
                                ctl_stack_move_amount = 3'b001;
                                ctl_stack_write = 1'b1;
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_3_LOW] = 1;
                    
                            end
                            
                            INST_JMP_A_DPTR : begin
                                ctl_run_ext_ext = 1'b1;
                                ctl_write_enable = 0;
                                ctl_da_counter = 1'b1;
                                ctl_DPTR_read  = 1'b1;
                            end
                            
                            INST_RET : begin
                                ctl_interruptable = 0;
                                ctl_run_ext_ext = 1;
                                ctl_write_enable = 0;
                                ctl_stack_move_amount = 3'b111;
                                ctl_stack_read = 1'b1;
                                                    
                            end
                            
                            INST_RETI : begin
                                ctl_interruptable = 0;
                                ctl_run_ext_ext = 1;
                                ctl_write_enable = 0;
                                ctl_stack_move_amount = 3'b111;
                                ctl_stack_read = 1'b1;
                                
                            end
                            
                            default : begin
                            
                            end
                            
                        endcase
                            
                    end 
                                    
                    current_state[S_RUN_EXT] : begin
                        
                        if (reg_jump_active) begin
                                next_state [S_INIT] = 1;
                                ctl_reset_I_fetch = 1'b1;
                                
                                ctl_ALU_active = 1; 
                                ctl_fetch_instruction_request = 1;
                                    
                                ctl_PC_cmd[OP_PC_SEQUENTIAL] = 1;
                                ctl_prefetch_cmd[OP_PREFETCH_SEQUENTIAL] = 1;
                                ctl_IR_cmd [OP_IR_LOAD_SEQUENTIAL] = 1;
                                    
                        end else if (!pipeline_stall) begin
                            next_state[S_RUN_NORMAL] = 1'b1;
                            
                            ctl_ALU_active = 1; 
                            ctl_fetch_instruction_request = 1;
                                
                            ctl_PC_cmd[OP_PC_SEQUENTIAL] = 1;
                            ctl_prefetch_cmd[OP_PREFETCH_SEQUENTIAL] = 1;
                            ctl_IR_cmd [OP_IR_LOAD_SEQUENTIAL] = 1;
                        
                        end else begin
                            next_state[S_RUN_EXT] = 1'b1;
                        end
                                            
                                   
                        ctl_interruptable = 0;
        
                        casex (IR[23 : 16])  // synthesis parallel_case 
                            
                            INST_MUL_AB : begin
                                ctl_ALU_cmd [OP_ALU_MUL_3RD] = 1;
                                ctl_load_A = 1;
                                ctl_PSW_flag_update = 1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_DIV_AB : begin
                                ctl_ALU_adjust_a = 1;
                                data_wr_address = B_ADDR;
                                ctl_PSW_flag_update = 1;
                            end
                            
                            {INST_XCHD_A_AT_R_MSB, 1'b?} : begin
                                ctl_load_A = 1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_DA_A : begin
                                ctl_load_A = 1;
                                ctl_ALU_cmd[OP_ALU_DA_WR] = 1'b1;
                                ctl_write_enable = 0;
                            end
                            
                            INST_SWAP_A : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_MOVC_A_DPTR : begin
                                ctl_ALU_adjust_b = 1;
                            end
                            
                            INST_MOVC_A_PC : begin
                                ctl_ALU_adjust_b = 1;
                            end
                            
                            INST_JC : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_JNC : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_JB : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_JNB : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_JBC : begin
                                
                            end             
                            
                            INST_JZ : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_JNZ : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_CJNE_A_DIR_REL : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_CJNE_A_DATA_REL : begin
                                ctl_write_enable = 0;
                            end
                            
                            {INST_CJNE_R_DATA_REL_MSB, 3'b???} : begin
                                ctl_write_enable = 0;
                            end
                            
                            {INST_CJNE_AT_R_DATA_REL_MSB, 1'b?} : begin
                                ctl_write_enable = 0;
                            end
                            
                            {INST_DJNZ_R_REL_MSB, 3'b???} : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_DJNZ_DIR_REL : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_SJMP : begin
                                ctl_write_enable = 0;
                            end
                            
                            {3'b???, INST_AJMP_LSB} : begin
                                ctl_write_enable = 0;
                            end
                            
                            {3'b???, INST_ACALL_LSB} : begin
                                ctl_stack_move_amount = 3'b001;
                                ctl_stack_write = 1'b1;
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_2_HIGH] = 1;
                        
                            end
                            
                            INST_LJMP : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_LCALL : begin
                                ctl_stack_move_amount = 3'b001;
                                ctl_stack_write = 1'b1;
                                ctl_ALU_adjust_b = 1;
                                ctl_ALU_cmd [OP_ALU_SAVE_PC_PLUS_3_HIGH] = 1;
                        
                            end
                            
                            INST_JMP_A_DPTR : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_RET : begin
                                ctl_write_enable = 0;
                            end
                            
                            INST_RETI : begin
                                ctl_write_enable = 0;
                                end
                            
                            default : begin
                                
                            end
                            
                        endcase
                        
                    end
                                        
                    current_state [S_RUN_EXT_EXT] : begin
                        
                        if (reg_jump_active) begin
                            next_state [S_INIT] = 1;
                            ctl_reset_I_fetch = 1'b1;
                            ctl_ALU_active = 1; 
                        end else if (!pipeline_stall) begin
                            ctl_ALU_active = 1; 
                            next_state [S_RUN_EXT] = 1;
                        end else begin
                            next_state [S_RUN_EXT_EXT] = 1;
                        end
                        
                        ctl_interruptable = 0;
                        
                        casex (IR[23 : 16])  // synthesis parallel_case 
                            INST_MUL_AB : begin
                                ctl_ALU_cmd [OP_ALU_MUL_2ND] = 1;
                                data_wr_address = B_ADDR;
                            end
                            
                            INST_DIV_AB : begin
                                
                                ctl_write_enable = 0;
                                
                                if ((wait_counter) && (!reg_jump_active)) begin
                                    next_state = 0;
                                    next_state [S_RUN_EXT_EXT] = 1;
                                    ctl_ALU_cmd [OP_ALU_DIV_SAVE_A] = 1;
                                end else begin
                                    ctl_ALU_cmd [OP_ALU_DIV_SAVE_B] = 1;
                                    ctl_load_A = 1;
                                end
                            end
                            
                            {INST_XCHD_A_AT_R_MSB, 1'b?} : begin
                                ctl_ALU_cmd [OP_ALU_MUL_2ND] = 1;
                                data_wr_address = active_Reg_address [7 : 0];
                                ctl_indirect_write = 1;
                            end
                                                    
                            INST_DA_A : begin
                                ctl_write_enable = 0;
                                    
                                if ((!wait_counter) || reg_jump_active) begin
                                    ctl_ALU_cmd [OP_ALU_MUL_2ND] = 1'b1;
                                end else begin
                                    ctl_ALU_cmd[OP_ALU_DA_SAVE] = 1'b1;
                                    next_state = 0;
                                    next_state [S_RUN_EXT_EXT] = 1;
                                end
                            end
                            
                            INST_MOVC_A_DPTR : begin
                                ctl_write_enable = 0;
                                
                                if ((wait_counter) && (!reg_jump_active)) begin
                                    next_state = 0;
                                    next_state [S_RUN_EXT_EXT] = 1;
                                end
                            end
                            
                            INST_MOVC_A_PC : begin
                                ctl_write_enable = 0;
                                
                                if ((wait_counter) && (!reg_jump_active)) begin
                                    next_state = 0;
                                    next_state [S_RUN_EXT_EXT] = 1;
                                end
                            end
                            
                            INST_JMP_A_DPTR : begin
                                ctl_write_enable = 0;
                                ctl_fetch_instruction_request = 1;
                                
                                if ((wait_counter) && (!reg_jump_active)) begin
                                    next_state = 0;
                                    next_state [S_RUN_EXT_EXT] = 1;
                                end
                            end
                            
                            INST_RET : begin
                                ctl_write_enable = 0;
                                ctl_stack_move_amount = 3'b111;
                                ctl_stack_read = 1'b1;
                                
                            end
                            
                            INST_RETI : begin
                                ctl_write_enable = 0;
                                ctl_stack_move_amount = 3'b111;
                                ctl_stack_read = 1'b1;
                                
                            end
                            
                            INST_SWAP_A : begin
                                ctl_ALU_cmd [OP_ALU_MUL_2ND] = 1;
                            end
                            
                            default : begin
                                
                            end
                            
                        endcase
                    end
                    
                    default: begin
                        next_state[S_INIT] = 1'b1;
                    end
                    
                endcase
            end
            
        end : state_machine_comb
    
    
endmodule : fast_core_control

`default_nettype wire
