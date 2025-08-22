################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modbus/modbus/rtu/mbcrc.c \
../Modbus/modbus/rtu/mbrtu.c \
../Modbus/modbus/rtu/mbrtu_m.c 

OBJS += \
./Modbus/modbus/rtu/mbcrc.o \
./Modbus/modbus/rtu/mbrtu.o \
./Modbus/modbus/rtu/mbrtu_m.o 

C_DEPS += \
./Modbus/modbus/rtu/mbcrc.d \
./Modbus/modbus/rtu/mbrtu.d \
./Modbus/modbus/rtu/mbrtu_m.d 


# Each subdirectory must supply rules for building sources it contributes
Modbus/modbus/rtu/%.o Modbus/modbus/rtu/%.su Modbus/modbus/rtu/%.cyclo: ../Modbus/modbus/rtu/%.c Modbus/modbus/rtu/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Modbus/port -I../Modbus/modbus -I../Modbus/port/rtt -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modbus-2f-modbus-2f-rtu

clean-Modbus-2f-modbus-2f-rtu:
	-$(RM) ./Modbus/modbus/rtu/mbcrc.cyclo ./Modbus/modbus/rtu/mbcrc.d ./Modbus/modbus/rtu/mbcrc.o ./Modbus/modbus/rtu/mbcrc.su ./Modbus/modbus/rtu/mbrtu.cyclo ./Modbus/modbus/rtu/mbrtu.d ./Modbus/modbus/rtu/mbrtu.o ./Modbus/modbus/rtu/mbrtu.su ./Modbus/modbus/rtu/mbrtu_m.cyclo ./Modbus/modbus/rtu/mbrtu_m.d ./Modbus/modbus/rtu/mbrtu_m.o ./Modbus/modbus/rtu/mbrtu_m.su

.PHONY: clean-Modbus-2f-modbus-2f-rtu

