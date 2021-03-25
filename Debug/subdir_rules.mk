################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.obj: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="F:/workspace-ccsv10/PLL-ISEE" --include_path="D:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/include" --include_path="D:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/lib" --include_path="D:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_common/include" --include_path="D:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_headers/include" --include_path="D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/include" --advice:performance=all --define=_LAUNCHXL_F28377S -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.obj: ../%.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: C2000 Compiler'
	"D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/bin/cl2000" -v28 -ml -mt --cla_support=cla1 --float_support=fpu32 --tmu_support=tmu0 --vcu_support=vcu2 --include_path="F:/workspace-ccsv10/PLL-ISEE" --include_path="D:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/include" --include_path="D:/ti/controlSUITE/libs/app_libs/solar/v1.2/float/lib" --include_path="D:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_common/include" --include_path="D:/ti/controlSUITE/device_support/F2837xS/v210/F2837xS_headers/include" --include_path="D:/ti/ccs1011/ccs/tools/compiler/ti-cgt-c2000_20.2.1.LTS/include" --advice:performance=all --define=_LAUNCHXL_F28377S -g --diag_warning=225 --diag_wrap=off --display_error_number --abi=coffabi --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


