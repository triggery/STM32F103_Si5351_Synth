################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../Core/Startup/startup_stm32f103c8tx.s 

OBJS += \
./Core/Startup/startup_stm32f103c8tx.o 

S_DEPS += \
./Core/Startup/startup_stm32f103c8tx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Startup/startup_stm32f103c8tx.o: ../Core/Startup/startup_stm32f103c8tx.s Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m3 -g3 -c -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/Si5351" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/SSD1306" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Core/Inc" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"/home/dmitry/Projects/STM32CubeIDE/STM32F103_Si5351_Synth/Drivers/CMSIS/Include" -x assembler-with-cpp -MMD -MP -MF"Core/Startup/startup_stm32f103c8tx.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@" "$<"

