file delete -force work
vlib work
vmap work work

set common "../../../common"
set pulserain_rtl_lib "../../PulseRain_rtl_lib"

vlog -work work -sv $pulserain_rtl_lib/BCD/DA_A.sv
vlog -work work -sv $pulserain_rtl_lib/SRT_DIV/SRT_Radix4_single_stage.sv
vlog -work work -sv $pulserain_rtl_lib/SRT_DIV/SRT_Radix4_cascade.sv
vlog -work work -sv $pulserain_rtl_lib/SRT_DIV/SRT_Radix4_division.sv



vlog -work work -sv $common/common_pkg.sv

vlog -work work -sv +incdir+$common +incdir+../source ../source/instruction_memory/IRAM_left.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/instruction_memory/IRAM_right.sv

vlog -work work -sv +incdir+$common +incdir+../source ../source/instruction_memory/Instruction_Memory.sv

vlog -work work -sv +incdir+$common +incdir+../source ../source/instruction_decode/Instruction_Size.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/instruction_decode/is_single_byte_instruction.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/instruction_decode/is_two_byte_instruction.sv


vlog -work work -sv +incdir+$common +incdir+../source ../source/pipeline/fast_core_ALU.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/pipeline/fast_core_control.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/pipeline/fast_core_I_fetch.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/pipeline/fast_core_pre_ALU.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/pipeline/fast_core_post_control.sv

vlog -work work -sv +incdir+$common +incdir+../source ../source/onchip_data/fast_core_onchip_data.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/onchip_data/fast_core_RAM.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/onchip_data/fast_core_reg_bank.sv
vlog -work work -sv +incdir+$common +incdir+../source ../source/onchip_data/fast_core_SFR.sv

vlog -work work -sv +incdir+$common +incdir+../source ../source/FP51_fast_core.sv
