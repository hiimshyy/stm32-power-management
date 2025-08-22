################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modbus/modbus/mb.c \
../Modbus/modbus/mb_m.c 

OBJS += \
./Modbus/modbus/mb.o \
./Modbus/modbus/mb_m.o 

C_DEPS += \
./Modbus/modbus/mb.d \
./Modbus/modbus/mb_m.d 


# Each subdirectory must supply rules for building sources it contributes
Modbus/modbus/%.o Modbus/modbus/%.su Modbus/modbus/%.cyclo: ../Modbus/modbus/%.c Modbus/modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Modbus/port -I../Modbus/modbus -I../Modbus/port/rtt -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modbus-2f-modbus

clean-Modbus-2f-modbus:
	-$(RM) ./Modbus/modbus/mb.cyclo ./Modbus/modbus/mb.d ./Modbus/modbus/mb.o ./Modbus/modbus/mb.su ./Modbus/modbus/mb_m.cyclo ./Modbus/modbus/mb_m.d ./Modbus/modbus/mb_m.o ./Modbus/modbus/mb_m.su

.PHONY: clean-Modbus-2f-modbus

