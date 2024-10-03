################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Core/Src/LCD.c \
../Drivers/Core/Src/RC_CAR.c \
../Drivers/Core/Src/Ultrasonic.c \
../Drivers/Core/Src/main.c \
../Drivers/Core/Src/stm32f4xx_hal_msp.c \
../Drivers/Core/Src/stm32f4xx_it.c \
../Drivers/Core/Src/syscalls.c \
../Drivers/Core/Src/sysmem.c \
../Drivers/Core/Src/system_stm32f4xx.c 

OBJS += \
./Drivers/Core/Src/LCD.o \
./Drivers/Core/Src/RC_CAR.o \
./Drivers/Core/Src/Ultrasonic.o \
./Drivers/Core/Src/main.o \
./Drivers/Core/Src/stm32f4xx_hal_msp.o \
./Drivers/Core/Src/stm32f4xx_it.o \
./Drivers/Core/Src/syscalls.o \
./Drivers/Core/Src/sysmem.o \
./Drivers/Core/Src/system_stm32f4xx.o 

C_DEPS += \
./Drivers/Core/Src/LCD.d \
./Drivers/Core/Src/RC_CAR.d \
./Drivers/Core/Src/Ultrasonic.d \
./Drivers/Core/Src/main.d \
./Drivers/Core/Src/stm32f4xx_hal_msp.d \
./Drivers/Core/Src/stm32f4xx_it.d \
./Drivers/Core/Src/syscalls.d \
./Drivers/Core/Src/sysmem.d \
./Drivers/Core/Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Core/Src/%.o Drivers/Core/Src/%.su Drivers/Core/Src/%.cyclo: ../Drivers/Core/Src/%.c Drivers/Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Core-2f-Src

clean-Drivers-2f-Core-2f-Src:
	-$(RM) ./Drivers/Core/Src/LCD.cyclo ./Drivers/Core/Src/LCD.d ./Drivers/Core/Src/LCD.o ./Drivers/Core/Src/LCD.su ./Drivers/Core/Src/RC_CAR.cyclo ./Drivers/Core/Src/RC_CAR.d ./Drivers/Core/Src/RC_CAR.o ./Drivers/Core/Src/RC_CAR.su ./Drivers/Core/Src/Ultrasonic.cyclo ./Drivers/Core/Src/Ultrasonic.d ./Drivers/Core/Src/Ultrasonic.o ./Drivers/Core/Src/Ultrasonic.su ./Drivers/Core/Src/main.cyclo ./Drivers/Core/Src/main.d ./Drivers/Core/Src/main.o ./Drivers/Core/Src/main.su ./Drivers/Core/Src/stm32f4xx_hal_msp.cyclo ./Drivers/Core/Src/stm32f4xx_hal_msp.d ./Drivers/Core/Src/stm32f4xx_hal_msp.o ./Drivers/Core/Src/stm32f4xx_hal_msp.su ./Drivers/Core/Src/stm32f4xx_it.cyclo ./Drivers/Core/Src/stm32f4xx_it.d ./Drivers/Core/Src/stm32f4xx_it.o ./Drivers/Core/Src/stm32f4xx_it.su ./Drivers/Core/Src/syscalls.cyclo ./Drivers/Core/Src/syscalls.d ./Drivers/Core/Src/syscalls.o ./Drivers/Core/Src/syscalls.su ./Drivers/Core/Src/sysmem.cyclo ./Drivers/Core/Src/sysmem.d ./Drivers/Core/Src/sysmem.o ./Drivers/Core/Src/sysmem.su ./Drivers/Core/Src/system_stm32f4xx.cyclo ./Drivers/Core/Src/system_stm32f4xx.d ./Drivers/Core/Src/system_stm32f4xx.o ./Drivers/Core/Src/system_stm32f4xx.su

.PHONY: clean-Drivers-2f-Core-2f-Src

