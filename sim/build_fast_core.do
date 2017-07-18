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


file delete -force work
vlib work
vmap work work

set common "../../../common"


vlog -work work -sv ../source/BCD/DA_A.sv
vlog -work work -sv ../source/SRT_DIV/SRT_Radix4_single_stage.sv
vlog -work work -sv ../source/SRT_DIV/SRT_Radix4_cascade.sv
vlog -work work -sv ../source/SRT_DIV/SRT_Radix4_division.sv


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
