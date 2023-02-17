################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
libraries/ucc5870/source/%.o: ../libraries/ucc5870/source/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -I"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_08_03_01_05/source" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/Motor" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/ucc5870/include" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/resolver/include" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/foc/include" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/filter/include" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/temp_sensor/include" -DSOC_AM263X -D_DEBUG_=1 -gstrict-dwarf -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"libraries/ucc5870/source/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/laptop/newworkspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


