################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -I"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_08_06_00_34/source" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/Motor" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/IpcComm" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/ucc5870/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/resolver/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/foc/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/filter/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/temp_sensor/include" -DSOC_AM263X -D_DEBUG_=1 -gstrict-dwarf -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-488624561: ../trinv.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"B:/ti/ccs1210/ccs/utils/sysconfig_1.16.1/sysconfig_cli.bat" -s "C:/ti/mcu_plus_sdk_am263x_08_06_00_34/.metadata/product.json" --script "C:/Users/Robi/workspace_v12/enet_cpsw_tcpserver_am263x-lp_r5fss0-1_freertos_ti-arm-clang/example.syscfg" --context "r5fss0-1" --script "C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/trinv.syscfg" --context "r5fss0-0" -o "syscfg" --part AM263x --package ZCZ --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_dpl_config.c: build-488624561 ../trinv.syscfg
syscfg/ti_dpl_config.h: build-488624561
syscfg/ti_drivers_config.c: build-488624561
syscfg/ti_drivers_config.h: build-488624561
syscfg/ti_drivers_open_close.c: build-488624561
syscfg/ti_drivers_open_close.h: build-488624561
syscfg/ti_pinmux_config.c: build-488624561
syscfg/ti_power_clock_config.c: build-488624561
syscfg/ti_board_config.c: build-488624561
syscfg/ti_board_config.h: build-488624561
syscfg/ti_board_open_close.c: build-488624561
syscfg/ti_board_open_close.h: build-488624561
syscfg/ti_enet_config.c: build-488624561
syscfg/ti_enet_config.h: build-488624561
syscfg/ti_enet_open_close.c: build-488624561
syscfg/ti_enet_open_close.h: build-488624561
syscfg/ti_enet_soc.c: build-488624561
syscfg/ti_enet_lwipif.c: build-488624561
syscfg/ti_enet_lwipif.h: build-488624561
syscfg/: build-488624561

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -I"C:/ti/ti_cgt_tiarmclang_1.3.1.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_08_06_00_34/source" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/Motor" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/IpcComm" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/ucc5870/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/resolver/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/foc/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/filter/include" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/libraries/temp_sensor/include" -DSOC_AM263X -D_DEBUG_=1 -gstrict-dwarf -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/Robi/workspace_v12/TractionDemo_am263x-cc_r5fss0-0_nortos_ti-arm-clang/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


