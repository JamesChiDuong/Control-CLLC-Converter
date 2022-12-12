################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../USER_LIB/Src/USER_Adc.c \
../USER_LIB/Src/USER_CALLBACK_FUNCTION.c \
../USER_LIB/Src/USER_Filter.c \
../USER_LIB/Src/USER_Flash.c \
../USER_LIB/Src/USER_Timer.c 

OBJS += \
./USER_LIB/Src/USER_Adc.o \
./USER_LIB/Src/USER_CALLBACK_FUNCTION.o \
./USER_LIB/Src/USER_Filter.o \
./USER_LIB/Src/USER_Flash.o \
./USER_LIB/Src/USER_Timer.o 

C_DEPS += \
./USER_LIB/Src/USER_Adc.d \
./USER_LIB/Src/USER_CALLBACK_FUNCTION.d \
./USER_LIB/Src/USER_Filter.d \
./USER_LIB/Src/USER_Flash.d \
./USER_LIB/Src/USER_Timer.d 


# Each subdirectory must supply rules for building sources it contributes
USER_LIB/Src/%.o USER_LIB/Src/%.su: ../USER_LIB/Src/%.c USER_LIB/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"D:/ChiDuong/STM32/MCUT_Electrical_Project/V1.1/USER_LIB/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-USER_LIB-2f-Src

clean-USER_LIB-2f-Src:
	-$(RM) ./USER_LIB/Src/USER_Adc.d ./USER_LIB/Src/USER_Adc.o ./USER_LIB/Src/USER_Adc.su ./USER_LIB/Src/USER_CALLBACK_FUNCTION.d ./USER_LIB/Src/USER_CALLBACK_FUNCTION.o ./USER_LIB/Src/USER_CALLBACK_FUNCTION.su ./USER_LIB/Src/USER_Filter.d ./USER_LIB/Src/USER_Filter.o ./USER_LIB/Src/USER_Filter.su ./USER_LIB/Src/USER_Flash.d ./USER_LIB/Src/USER_Flash.o ./USER_LIB/Src/USER_Flash.su ./USER_LIB/Src/USER_Timer.d ./USER_LIB/Src/USER_Timer.o ./USER_LIB/Src/USER_Timer.su

.PHONY: clean-USER_LIB-2f-Src

