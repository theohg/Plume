################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/TCA9548_Driver/src/TCA9548.cpp 

OBJS += \
./Drivers/TCA9548_Driver/src/TCA9548.o 

CPP_DEPS += \
./Drivers/TCA9548_Driver/src/TCA9548.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/TCA9548_Driver/src/%.o Drivers/TCA9548_Driver/src/%.su Drivers/TCA9548_Driver/src/%.cyclo: ../Drivers/TCA9548_Driver/src/%.cpp Drivers/TCA9548_Driver/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/DRV8214_Driver/include" -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/TCA9548_Driver/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-TCA9548_Driver-2f-src

clean-Drivers-2f-TCA9548_Driver-2f-src:
	-$(RM) ./Drivers/TCA9548_Driver/src/TCA9548.cyclo ./Drivers/TCA9548_Driver/src/TCA9548.d ./Drivers/TCA9548_Driver/src/TCA9548.o ./Drivers/TCA9548_Driver/src/TCA9548.su

.PHONY: clean-Drivers-2f-TCA9548_Driver-2f-src

