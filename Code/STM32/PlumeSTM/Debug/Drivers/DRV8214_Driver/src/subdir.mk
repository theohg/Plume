################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/DRV8214_Driver/src/drv8214.cpp \
../Drivers/DRV8214_Driver/src/drv8214_platform_i2c.cpp 

OBJS += \
./Drivers/DRV8214_Driver/src/drv8214.o \
./Drivers/DRV8214_Driver/src/drv8214_platform_i2c.o 

CPP_DEPS += \
./Drivers/DRV8214_Driver/src/drv8214.d \
./Drivers/DRV8214_Driver/src/drv8214_platform_i2c.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/DRV8214_Driver/src/%.o Drivers/DRV8214_Driver/src/%.su Drivers/DRV8214_Driver/src/%.cyclo: ../Drivers/DRV8214_Driver/src/%.cpp Drivers/DRV8214_Driver/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/DRV8214_Driver/include" -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/TCA9548_Driver/include" -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/BMI270_Driver/include" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-DRV8214_Driver-2f-src

clean-Drivers-2f-DRV8214_Driver-2f-src:
	-$(RM) ./Drivers/DRV8214_Driver/src/drv8214.cyclo ./Drivers/DRV8214_Driver/src/drv8214.d ./Drivers/DRV8214_Driver/src/drv8214.o ./Drivers/DRV8214_Driver/src/drv8214.su ./Drivers/DRV8214_Driver/src/drv8214_platform_i2c.cyclo ./Drivers/DRV8214_Driver/src/drv8214_platform_i2c.d ./Drivers/DRV8214_Driver/src/drv8214_platform_i2c.o ./Drivers/DRV8214_Driver/src/drv8214_platform_i2c.su

.PHONY: clean-Drivers-2f-DRV8214_Driver-2f-src

