################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/CC1201_commands.c \
../Core/Src/CC1201_detection.c \
../Core/Src/CC1201_hardware_test.c \
../Core/Src/CC1201_simple_link_reg_config.c \
../Core/Src/STM32_pin_diagnostic.c \
../Core/Src/main.c \
../Core/Src/stm32h7xx_hal_msp.c \
../Core/Src/stm32h7xx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32h7xx.c 

OBJS += \
./Core/Src/CC1201_commands.o \
./Core/Src/CC1201_detection.o \
./Core/Src/CC1201_hardware_test.o \
./Core/Src/CC1201_simple_link_reg_config.o \
./Core/Src/STM32_pin_diagnostic.o \
./Core/Src/main.o \
./Core/Src/stm32h7xx_hal_msp.o \
./Core/Src/stm32h7xx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32h7xx.o 

C_DEPS += \
./Core/Src/CC1201_commands.d \
./Core/Src/CC1201_detection.d \
./Core/Src/CC1201_hardware_test.d \
./Core/Src/CC1201_simple_link_reg_config.d \
./Core/Src/STM32_pin_diagnostic.d \
./Core/Src/main.d \
./Core/Src/stm32h7xx_hal_msp.d \
./Core/Src/stm32h7xx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32h7xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32H7xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/CC1201_commands.cyclo ./Core/Src/CC1201_commands.d ./Core/Src/CC1201_commands.o ./Core/Src/CC1201_commands.su ./Core/Src/CC1201_detection.cyclo ./Core/Src/CC1201_detection.d ./Core/Src/CC1201_detection.o ./Core/Src/CC1201_detection.su ./Core/Src/CC1201_hardware_test.cyclo ./Core/Src/CC1201_hardware_test.d ./Core/Src/CC1201_hardware_test.o ./Core/Src/CC1201_hardware_test.su ./Core/Src/CC1201_simple_link_reg_config.cyclo ./Core/Src/CC1201_simple_link_reg_config.d ./Core/Src/CC1201_simple_link_reg_config.o ./Core/Src/CC1201_simple_link_reg_config.su ./Core/Src/STM32_pin_diagnostic.cyclo ./Core/Src/STM32_pin_diagnostic.d ./Core/Src/STM32_pin_diagnostic.o ./Core/Src/STM32_pin_diagnostic.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32h7xx_hal_msp.cyclo ./Core/Src/stm32h7xx_hal_msp.d ./Core/Src/stm32h7xx_hal_msp.o ./Core/Src/stm32h7xx_hal_msp.su ./Core/Src/stm32h7xx_it.cyclo ./Core/Src/stm32h7xx_it.d ./Core/Src/stm32h7xx_it.o ./Core/Src/stm32h7xx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32h7xx.cyclo ./Core/Src/system_stm32h7xx.d ./Core/Src/system_stm32h7xx.o ./Core/Src/system_stm32h7xx.su

.PHONY: clean-Core-2f-Src

