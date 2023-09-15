################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1240/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="C:/neilrw2_dkim305/ME461_repo/workspace/lab 2 Ex2" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/device_support/f2837xd/common/include" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/device_support/f2837xd/headers/include" --include_path="C:/neilrw2_dkim305/ME461_repo/workspace/lab 2 Ex2/device" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/libraries/dsp/FPU/c28/include" --include_path="C:/ti/ccs1240/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --define=DEBUG --define=_DUAL_HEADERS --define=CPU1 --define=_LAUNCHXL_F28379D --c11 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"C:/ti/ccs1240/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 -Ooff --include_path="C:/neilrw2_dkim305/ME461_repo/workspace/lab 2 Ex2" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/device_support/f2837xd/common/include" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/device_support/f2837xd/headers/include" --include_path="C:/neilrw2_dkim305/ME461_repo/workspace/lab 2 Ex2/device" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/driverlib/f2837xd/driverlib" --include_path="C:/neilrw2_dkim305/ME461_repo/C2000Ware_4_01_00_00/libraries/dsp/FPU/c28/include" --include_path="C:/ti/ccs1240/ccs/tools/compiler/ti-cgt-c2000_22.6.0.LTS/include" --define=DEBUG --define=_DUAL_HEADERS --define=CPU1 --define=_LAUNCHXL_F28379D --c11 --diag_suppress=10063 --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


