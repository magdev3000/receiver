################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/SX1278/SX1278.c \
../Core/SX1278/SX1278_hw.c 

OBJS += \
./Core/SX1278/SX1278.o \
./Core/SX1278/SX1278_hw.o 

C_DEPS += \
./Core/SX1278/SX1278.d \
./Core/SX1278/SX1278_hw.d 


# Each subdirectory must supply rules for building sources it contributes
Core/SX1278/SX1278.o: ../Core/SX1278/SX1278.c Core/SX1278/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/czq/STM32CubeIDE/workspace_1.5.0/Receiver/Core/SX1278" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/SX1278/SX1278.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"
Core/SX1278/SX1278_hw.o: ../Core/SX1278/SX1278_hw.c Core/SX1278/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L053xx -c -I../Core/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc -I../Drivers/STM32L0xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L0xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/czq/STM32CubeIDE/workspace_1.5.0/Receiver/Core/SX1278" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Core/SX1278/SX1278_hw.d" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

