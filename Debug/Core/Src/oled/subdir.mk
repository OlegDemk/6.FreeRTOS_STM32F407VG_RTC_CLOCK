################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/oled/fonts.c \
../Core/Src/oled/gfx.c \
../Core/Src/oled/oled.c 

OBJS += \
./Core/Src/oled/fonts.o \
./Core/Src/oled/gfx.o \
./Core/Src/oled/oled.o 

C_DEPS += \
./Core/Src/oled/fonts.d \
./Core/Src/oled/gfx.d \
./Core/Src/oled/oled.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/oled/%.o: ../Core/Src/oled/%.c Core/Src/oled/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-oled

clean-Core-2f-Src-2f-oled:
	-$(RM) ./Core/Src/oled/fonts.d ./Core/Src/oled/fonts.o ./Core/Src/oled/gfx.d ./Core/Src/oled/gfx.o ./Core/Src/oled/oled.d ./Core/Src/oled/oled.o

.PHONY: clean-Core-2f-Src-2f-oled

