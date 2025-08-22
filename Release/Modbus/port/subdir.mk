################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modbus/port/user_mb_app.c \
../Modbus/port/user_mb_app_m.c 

OBJS += \
./Modbus/port/user_mb_app.o \
./Modbus/port/user_mb_app_m.o 

C_DEPS += \
./Modbus/port/user_mb_app.d \
./Modbus/port/user_mb_app_m.d 


# Each subdirectory must supply rules for building sources it contributes
Modbus/port/%.o Modbus/port/%.su Modbus/port/%.cyclo: ../Modbus/port/%.c Modbus/port/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Modbus/port -I../Modbus/modbus -I../Modbus/port/rtt -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modbus-2f-port

clean-Modbus-2f-port:
	-$(RM) ./Modbus/port/user_mb_app.cyclo ./Modbus/port/user_mb_app.d ./Modbus/port/user_mb_app.o ./Modbus/port/user_mb_app.su ./Modbus/port/user_mb_app_m.cyclo ./Modbus/port/user_mb_app_m.d ./Modbus/port/user_mb_app_m.o ./Modbus/port/user_mb_app_m.su

.PHONY: clean-Modbus-2f-port

