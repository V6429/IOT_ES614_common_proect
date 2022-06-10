################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lora/LoRa.c 

OBJS += \
./Core/lora/LoRa.o 

C_DEPS += \
./Core/lora/LoRa.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lora/%.o Core/lora/%.su: ../Core/lora/%.c Core/lora/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Victor/Desktop/MTECH/sem-2/IOT-ES614/LAB_teamWS/node_2_stm32/stm_cubeide_ws/Core/lora/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lora

clean-Core-2f-lora:
	-$(RM) ./Core/lora/LoRa.d ./Core/lora/LoRa.o ./Core/lora/LoRa.su

.PHONY: clean-Core-2f-lora

