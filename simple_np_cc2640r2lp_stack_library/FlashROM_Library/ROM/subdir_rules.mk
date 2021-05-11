################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
ROM/common_rom_init.obj: D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/rom/r2/common_rom_init.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/ProgramFiles/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" --cmd_file="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/config/build_components.opt" --cmd_file="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/config/factory_config.opt" --cmd_file="D:/Drawer/CodeComposerStudioFiles/I_baseball/simple_np_cc2640r2lp_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="D:/Drawer/CodeComposerStudioFiles/I_baseball/simple_np_cc2640r2lp_stack_library" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/rom" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/common/cc26xx" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/common/cc26xx/npi/stack" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/examples/rtos/CC2640R2_LAUNCHXL/blestack/simple_np/src/stack" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/icall/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/profiles/roles" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/target/_common" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/target" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/icall/src/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/npi/src" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/osal/src/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/services/src/nv" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/services/src/nv/cc26xx" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/services/src/saddr" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2/rf_patches" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages" --include_path="D:/ProgramFiles/ti/xdctools_3_50_03_33_core/packages" --include_path="D:/ProgramFiles/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=GAP_BONDINGS_MAX=16 --define=GAP_CHAR_CFG_MAX=16 --define=GATT_MAX_NUM_PREPARE_WRITES=24 --define=GATT_NO_CLIENT --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=3 --define=OSAL_SNV=1 --define=POWER_SAVING --define=RF_SINGLEMODE --define=SNP_SECURITY --define=STACK_LIBRARY --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="ROM/common_rom_init.d_raw" --obj_directory="ROM" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

ROM/rom_init.obj: D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/rom/r2/rom_init.c $(GEN_OPTS) | $(GEN_HDRS)
	@echo 'Building file: "$<"'
	@echo 'Invoking: ARM Compiler'
	"D:/ProgramFiles/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/bin/armcl" --cmd_file="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/config/build_components.opt" --cmd_file="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/config/factory_config.opt" --cmd_file="D:/Drawer/CodeComposerStudioFiles/I_baseball/simple_np_cc2640r2lp_stack_library/TOOLS/build_config.opt"  -mv7M3 --code_state=16 -me -O4 --opt_for_speed=0 --include_path="D:/Drawer/CodeComposerStudioFiles/I_baseball/simple_np_cc2640r2lp_stack_library" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/controller/cc26xx_r2/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/rom" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/common/cc26xx" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/common/cc26xx/npi/stack" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/examples/rtos/CC2640R2_LAUNCHXL/blestack/simple_np/src/stack" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/icall/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/profiles/roles" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/target/_common" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/target/_common/cc26xx" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/hal/src/target" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/icall/src/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/npi/src" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/osal/src/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/services/src/nv" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/services/src/nv/cc26xx" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/blestack/services/src/saddr" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2/rf_patches" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source/ti/devices/cc26x0r2/inc" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/source" --include_path="D:/ProgramFiles/ti/simplelink_cc2640r2_sdk_1_50_00_58/kernel/tirtos/packages" --include_path="D:/ProgramFiles/ti/xdctools_3_50_03_33_core/packages" --include_path="D:/ProgramFiles/ti/ccsv7/tools/compiler/ti-cgt-arm_16.9.6.LTS/include" --define=CC26XX --define=CC26XX_R2 --define=DeviceFamily_CC26X0R2 --define=EXT_HAL_ASSERT --define=FLASH_ROM_BUILD --define=GAP_BONDINGS_MAX=16 --define=GAP_CHAR_CFG_MAX=16 --define=GATT_MAX_NUM_PREPARE_WRITES=24 --define=GATT_NO_CLIENT --define=ICALL_EVENTS --define=ICALL_JT --define=ICALL_LITE --define=OSAL_CBTIMER_NUM_TASKS=1 --define=OSAL_MAX_NUM_PROXY_TASKS=3 --define=OSAL_SNV=1 --define=POWER_SAVING --define=RF_SINGLEMODE --define=SNP_SECURITY --define=STACK_LIBRARY --define=USE_ICALL -g --c99 --gcc --diag_warning=225 --diag_wrap=off --display_error_number --gen_func_subsections=on --abi=eabi --preproc_with_compile --preproc_dependency="ROM/rom_init.d_raw" --obj_directory="ROM" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


