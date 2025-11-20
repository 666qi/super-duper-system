################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla2 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu0 --fp_mode=strict --include_path="D:/Desktop/Single-phase_inverter/User" --include_path="D:/Desktop/Single-phase_inverter/spll" --include_path="D:/Desktop/Single-phase_inverter/globalvars" --include_path="D:/Desktop/Single-phase_inverter/PID" --include_path="D:/Desktop/Single-phase_inverter/ControlStrategy" --include_path="D:/Desktop/Single-phase_inverter" --include_path="D:/Desktop/Single-phase_inverter/spwm" --include_path="D:/Desktop/Single-phase_inverter/common/include" --include_path="D:/Desktop/Single-phase_inverter/headers/include" --include_path="D:/Desktop/Single-phase_inverter/board" --include_path="D:/ccs/tools/compiler/ti-cgt-c2000_22.6.1.LTS/include" --advice:performance=all --define=_FLASH -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


