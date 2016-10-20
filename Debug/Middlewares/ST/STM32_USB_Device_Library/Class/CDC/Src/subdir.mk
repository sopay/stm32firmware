################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c 

OBJS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.o 

C_DEPS += \
./Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.d 


# Each subdirectory must supply rules for building sources it contributes
Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.o: ../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -D__weak="__attribute__((weak))" -D__packed="__attribute__((__packed__))" -DUSE_HAL_DRIVER -DSTM32F746xx -I"/Users/milgerl/Documents/workspace/stm32firmware/Inc" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/CMSIS/Include" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/STM32F7xx_HAL_Driver/Inc" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy" -I"/Users/milgerl/Documents/workspace/stm32firmware/Middlewares/ST/STM32_USB_Device_Library/Core/Inc" -I"/Users/milgerl/Documents/workspace/stm32firmware/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/CMSIS/Device/ST/STM32F7xx/Include" -I"/Users/milgerl/Documents/workspace/stm32firmware/GUI" -I"/Users/milgerl/Documents/workspace/stm32firmware/Src" -I"/Users/milgerl/Documents/workspace/stm32firmware/GUI/inc" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/Utilities/Fonts" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/Utilities" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/Components/rk043fn48h" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/Components/ft5336" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/Components" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/Common" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/BSP/src" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/BSP/inc" -I"/Users/milgerl/Documents/workspace/stm32firmware/Drivers/BSP" -I"/Users/milgerl/Documents/workspace/stm32firmware/GUI/Lib"  -Os -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


