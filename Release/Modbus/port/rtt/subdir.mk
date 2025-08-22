################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modbus/port/rtt/port.c \
../Modbus/port/rtt/portevent.c \
../Modbus/port/rtt/portevent_m.c \
../Modbus/port/rtt/portserial.c \
../Modbus/port/rtt/portserial_m.c \
../Modbus/port/rtt/porttimer.c \
../Modbus/port/rtt/porttimer_m.c 

OBJS += \
./Modbus/port/rtt/port.o \
./Modbus/port/rtt/portevent.o \
./Modbus/port/rtt/portevent_m.o \
./Modbus/port/rtt/portserial.o \
./Modbus/port/rtt/portserial_m.o \
./Modbus/port/rtt/porttimer.o \
./Modbus/port/rtt/porttimer_m.o 

C_DEPS += \
./Modbus/port/rtt/port.d \
./Modbus/port/rtt/portevent.d \
./Modbus/port/rtt/portevent_m.d \
./Modbus/port/rtt/portserial.d \
./Modbus/port/rtt/portserial_m.d \
./Modbus/port/rtt/porttimer.d \
./Modbus/port/rtt/porttimer_m.d 


# Each subdirectory must supply rules for building sources it contributes
Modbus/port/rtt/%.o Modbus/port/rtt/%.su Modbus/port/rtt/%.cyclo: ../Modbus/port/rtt/%.c Modbus/port/rtt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Modbus/port -I../Modbus/modbus -I../Modbus/port/rtt -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modbus-2f-port-2f-rtt

clean-Modbus-2f-port-2f-rtt:
	-$(RM) ./Modbus/port/rtt/port.cyclo ./Modbus/port/rtt/port.d ./Modbus/port/rtt/port.o ./Modbus/port/rtt/port.su ./Modbus/port/rtt/portevent.cyclo ./Modbus/port/rtt/portevent.d ./Modbus/port/rtt/portevent.o ./Modbus/port/rtt/portevent.su ./Modbus/port/rtt/portevent_m.cyclo ./Modbus/port/rtt/portevent_m.d ./Modbus/port/rtt/portevent_m.o ./Modbus/port/rtt/portevent_m.su ./Modbus/port/rtt/portserial.cyclo ./Modbus/port/rtt/portserial.d ./Modbus/port/rtt/portserial.o ./Modbus/port/rtt/portserial.su ./Modbus/port/rtt/portserial_m.cyclo ./Modbus/port/rtt/portserial_m.d ./Modbus/port/rtt/portserial_m.o ./Modbus/port/rtt/portserial_m.su ./Modbus/port/rtt/porttimer.cyclo ./Modbus/port/rtt/porttimer.d ./Modbus/port/rtt/porttimer.o ./Modbus/port/rtt/porttimer.su ./Modbus/port/rtt/porttimer_m.cyclo ./Modbus/port/rtt/porttimer_m.d ./Modbus/port/rtt/porttimer_m.o ./Modbus/port/rtt/porttimer_m.su

.PHONY: clean-Modbus-2f-port-2f-rtt

