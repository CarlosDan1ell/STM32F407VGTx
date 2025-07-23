################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Driverss/Src/stm32f4_discovery_audio.c 

OBJS += \
./Driverss/Src/stm32f4_discovery_audio.o 

C_DEPS += \
./Driverss/Src/stm32f4_discovery_audio.d 


# Each subdirectory must supply rules for building sources it contributes
Driverss/Src/%.o Driverss/Src/%.su Driverss/Src/%.cyclo: ../Driverss/Src/%.c Driverss/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I../PDM2PCM/App -I../Middlewares/ST/STM32_Audio/Addons/PDM/Inc -I"C:/Users/Kit-covid-0739/Desktop/Firmwareee/STM32x/STM32F407VGxxx/STM32F407VGx/Driverss/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Driverss-2f-Src

clean-Driverss-2f-Src:
	-$(RM) ./Driverss/Src/stm32f4_discovery_audio.cyclo ./Driverss/Src/stm32f4_discovery_audio.d ./Driverss/Src/stm32f4_discovery_audio.o ./Driverss/Src/stm32f4_discovery_audio.su

.PHONY: clean-Driverss-2f-Src

