################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/SSD1306/fonts.c \
../Drivers/SSD1306/ssd1306.c 

OBJS += \
./Drivers/SSD1306/fonts.o \
./Drivers/SSD1306/ssd1306.o 

C_DEPS += \
./Drivers/SSD1306/fonts.d \
./Drivers/SSD1306/ssd1306.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/SSD1306/fonts.o: ../Drivers/SSD1306/fonts.c Drivers/SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/Si5351" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/SSD1306" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Include" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/SSD1306/fonts.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Drivers/SSD1306/ssd1306.o: ../Drivers/SSD1306/ssd1306.c Drivers/SSD1306/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/Si5351" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/SSD1306" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Include" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Core/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Drivers/SSD1306/ssd1306.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

