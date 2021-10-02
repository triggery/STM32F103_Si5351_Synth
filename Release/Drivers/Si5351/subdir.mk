################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Si5351/si5351.c 

OBJS += \
./Drivers/Si5351/si5351.o 

C_DEPS += \
./Drivers/Si5351/si5351.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Si5351/si5351.o: ../Drivers/Si5351/si5351.c Drivers/Si5351/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/Si5351" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/SSD1306" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Core/Inc" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Include" -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/Keyboard" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/Si5351/si5351.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

