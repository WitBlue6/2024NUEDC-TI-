################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"E:/ti/ccstheia141/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1" -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1/OLED" -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1/Debug" -I"E:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/DSP/Include" -I"E:/ti/mspm0_sdk_2_01_00_03/source" -I"E:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-1685977823: ../adc_timer_dma_1.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"E:/ti/ccstheia141/ccs/utils/sysconfig_1.20.0/sysconfig_cli.bat" --script "C:/Users/he/workspace_ccstheia/adc_timer_dma_1/adc_timer_dma_1.syscfg" -o "." -s "E:/ti/mspm0_sdk_2_01_00_03/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-1685977823 ../adc_timer_dma_1.syscfg
device.opt: build-1685977823
device.cmd.genlibs: build-1685977823
ti_msp_dl_config.c: build-1685977823
ti_msp_dl_config.h: build-1685977823
Event.dot: build-1685977823

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"E:/ti/ccstheia141/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1" -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1/OLED" -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1/Debug" -I"E:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/DSP/Include" -I"E:/ti/mspm0_sdk_2_01_00_03/source" -I"E:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: E:/ti/mspm0_sdk_2_01_00_03/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"E:/ti/ccstheia141/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1" -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1/OLED" -I"C:/Users/he/workspace_ccstheia/adc_timer_dma_1/Debug" -I"E:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/DSP/Include" -I"E:/ti/mspm0_sdk_2_01_00_03/source" -I"E:/ti/mspm0_sdk_2_01_00_03/source/third_party/CMSIS/Core/Include" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


