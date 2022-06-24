################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/lora/src/LoRa.c 

OBJS += \
./Core/lora/src/LoRa.o 

C_DEPS += \
./Core/lora/src/LoRa.d 


# Each subdirectory must supply rules for building sources it contributes
Core/lora/src/%.o Core/lora/src/%.su: ../Core/lora/src/%.c Core/lora/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F401xE -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Victor/Desktop/MTECH/sem-2/IOT-ES614/LAB_teamWS/node_2_stm32/stm_cubeide_ws/Core/lora/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-lora-2f-src

clean-Core-2f-lora-2f-src:
	-$(RM) ./Core/lora/src/LoRa.d ./Core/lora/src/LoRa.o ./Core/lora/src/LoRa.su

.PHONY: clean-Core-2f-lora-2f-src

