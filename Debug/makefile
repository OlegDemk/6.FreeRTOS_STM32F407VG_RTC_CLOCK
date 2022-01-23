################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include USB_DEVICE/Target/subdir.mk
-include USB_DEVICE/App/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Core/Src/subdir.mk
-include Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/oled/subdir.mk
-include Core/Src/clock/subdir.mk
-include Core/Src/subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := 6.FreeRTOS_STM32F407VG_RTC_CLOCK
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
6.FreeRTOS_STM32F407VG_RTC_CLOCK.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
6.FreeRTOS_STM32F407VG_RTC_CLOCK.list \

OBJCOPY_BIN += \
6.FreeRTOS_STM32F407VG_RTC_CLOCK.bin \


# All Target
all: main-build

# Main-build Target
main-build: 6.FreeRTOS_STM32F407VG_RTC_CLOCK.elf secondary-outputs

# Tool invocations
6.FreeRTOS_STM32F407VG_RTC_CLOCK.elf: $(OBJS) $(USER_OBJS) /home/odemki/my_files/my_projects/STM32/FreeRTOS/6.FreeRTOS_STM32F407VG_RTC_CLOCK/STM32F407VGTX_FLASH.ld makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-gcc -o "6.FreeRTOS_STM32F407VG_RTC_CLOCK.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/home/odemki/my_files/my_projects/STM32/FreeRTOS/6.FreeRTOS_STM32F407VG_RTC_CLOCK/STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="6.FreeRTOS_STM32F407VG_RTC_CLOCK.map" -Wl,--gc-sections -static -u_print_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -u _printf_float -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

6.FreeRTOS_STM32F407VG_RTC_CLOCK.list: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "6.FreeRTOS_STM32F407VG_RTC_CLOCK.list"
	@echo 'Finished building: $@'
	@echo ' '

6.FreeRTOS_STM32F407VG_RTC_CLOCK.bin: $(EXECUTABLES) makefile objects.list $(OPTIONAL_TOOL_DEPS)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "6.FreeRTOS_STM32F407VG_RTC_CLOCK.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) 6.FreeRTOS_STM32F407VG_RTC_CLOCK.bin 6.FreeRTOS_STM32F407VG_RTC_CLOCK.elf 6.FreeRTOS_STM32F407VG_RTC_CLOCK.list default.size.stdout
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

fail-specified-linker-script-missing:
	@echo 'Error: Cannot find the specified linker script. Check the linker settings in the build configuration.'
	@exit 2

warn-no-linker-script-specified:
	@echo 'Warning: No linker script specified. Check the linker settings in the build configuration.'

.PHONY: all clean dependents main-build fail-specified-linker-script-missing warn-no-linker-script-specified

-include ../makefile.targets
