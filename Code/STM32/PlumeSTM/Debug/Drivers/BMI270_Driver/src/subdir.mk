################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BMI270_Driver/src/bmi2.c \
../Drivers/BMI270_Driver/src/bmi270.c \
../Drivers/BMI270_Driver/src/bmi2_user_interface.c 

C_DEPS += \
./Drivers/BMI270_Driver/src/bmi2.d \
./Drivers/BMI270_Driver/src/bmi270.d \
./Drivers/BMI270_Driver/src/bmi2_user_interface.d 

OBJS += \
./Drivers/BMI270_Driver/src/bmi2.o \
./Drivers/BMI270_Driver/src/bmi270.o \
./Drivers/BMI270_Driver/src/bmi2_user_interface.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BMI270_Driver/src/%.o Drivers/BMI270_Driver/src/%.su Drivers/BMI270_Driver/src/%.cyclo: ../Drivers/BMI270_Driver/src/%.c Drivers/BMI270_Driver/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB5Mxx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/DRV8214_Driver/include" -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/TCA9548_Driver/include" -I"/Users/theoh/Documents/EPFL/MA4/semester_project/Plume/Code/STM32/PlumeSTM/Drivers/BMI270_Driver/include" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BMI270_Driver-2f-src

clean-Drivers-2f-BMI270_Driver-2f-src:
	-$(RM) ./Drivers/BMI270_Driver/src/bmi2.cyclo ./Drivers/BMI270_Driver/src/bmi2.d ./Drivers/BMI270_Driver/src/bmi2.o ./Drivers/BMI270_Driver/src/bmi2.su ./Drivers/BMI270_Driver/src/bmi270.cyclo ./Drivers/BMI270_Driver/src/bmi270.d ./Drivers/BMI270_Driver/src/bmi270.o ./Drivers/BMI270_Driver/src/bmi270.su ./Drivers/BMI270_Driver/src/bmi2_user_interface.cyclo ./Drivers/BMI270_Driver/src/bmi2_user_interface.d ./Drivers/BMI270_Driver/src/bmi2_user_interface.o ./Drivers/BMI270_Driver/src/bmi2_user_interface.su

.PHONY: clean-Drivers-2f-BMI270_Driver-2f-src

