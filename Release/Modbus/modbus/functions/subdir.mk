################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Modbus/modbus/functions/mbfunccoils.c \
../Modbus/modbus/functions/mbfunccoils_m.c \
../Modbus/modbus/functions/mbfuncdiag.c \
../Modbus/modbus/functions/mbfuncdisc.c \
../Modbus/modbus/functions/mbfuncdisc_m.c \
../Modbus/modbus/functions/mbfuncholding.c \
../Modbus/modbus/functions/mbfuncholding_m.c \
../Modbus/modbus/functions/mbfuncinput.c \
../Modbus/modbus/functions/mbfuncinput_m.c \
../Modbus/modbus/functions/mbfuncother.c \
../Modbus/modbus/functions/mbfuncutils.c 

OBJS += \
./Modbus/modbus/functions/mbfunccoils.o \
./Modbus/modbus/functions/mbfunccoils_m.o \
./Modbus/modbus/functions/mbfuncdiag.o \
./Modbus/modbus/functions/mbfuncdisc.o \
./Modbus/modbus/functions/mbfuncdisc_m.o \
./Modbus/modbus/functions/mbfuncholding.o \
./Modbus/modbus/functions/mbfuncholding_m.o \
./Modbus/modbus/functions/mbfuncinput.o \
./Modbus/modbus/functions/mbfuncinput_m.o \
./Modbus/modbus/functions/mbfuncother.o \
./Modbus/modbus/functions/mbfuncutils.o 

C_DEPS += \
./Modbus/modbus/functions/mbfunccoils.d \
./Modbus/modbus/functions/mbfunccoils_m.d \
./Modbus/modbus/functions/mbfuncdiag.d \
./Modbus/modbus/functions/mbfuncdisc.d \
./Modbus/modbus/functions/mbfuncdisc_m.d \
./Modbus/modbus/functions/mbfuncholding.d \
./Modbus/modbus/functions/mbfuncholding_m.d \
./Modbus/modbus/functions/mbfuncinput.d \
./Modbus/modbus/functions/mbfuncinput_m.d \
./Modbus/modbus/functions/mbfuncother.d \
./Modbus/modbus/functions/mbfuncutils.d 


# Each subdirectory must supply rules for building sources it contributes
Modbus/modbus/functions/%.o Modbus/modbus/functions/%.su Modbus/modbus/functions/%.cyclo: ../Modbus/modbus/functions/%.c Modbus/modbus/functions/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Modbus/port -I../Modbus/modbus -I../Modbus/port/rtt -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Modbus-2f-modbus-2f-functions

clean-Modbus-2f-modbus-2f-functions:
	-$(RM) ./Modbus/modbus/functions/mbfunccoils.cyclo ./Modbus/modbus/functions/mbfunccoils.d ./Modbus/modbus/functions/mbfunccoils.o ./Modbus/modbus/functions/mbfunccoils.su ./Modbus/modbus/functions/mbfunccoils_m.cyclo ./Modbus/modbus/functions/mbfunccoils_m.d ./Modbus/modbus/functions/mbfunccoils_m.o ./Modbus/modbus/functions/mbfunccoils_m.su ./Modbus/modbus/functions/mbfuncdiag.cyclo ./Modbus/modbus/functions/mbfuncdiag.d ./Modbus/modbus/functions/mbfuncdiag.o ./Modbus/modbus/functions/mbfuncdiag.su ./Modbus/modbus/functions/mbfuncdisc.cyclo ./Modbus/modbus/functions/mbfuncdisc.d ./Modbus/modbus/functions/mbfuncdisc.o ./Modbus/modbus/functions/mbfuncdisc.su ./Modbus/modbus/functions/mbfuncdisc_m.cyclo ./Modbus/modbus/functions/mbfuncdisc_m.d ./Modbus/modbus/functions/mbfuncdisc_m.o ./Modbus/modbus/functions/mbfuncdisc_m.su ./Modbus/modbus/functions/mbfuncholding.cyclo ./Modbus/modbus/functions/mbfuncholding.d ./Modbus/modbus/functions/mbfuncholding.o ./Modbus/modbus/functions/mbfuncholding.su ./Modbus/modbus/functions/mbfuncholding_m.cyclo ./Modbus/modbus/functions/mbfuncholding_m.d ./Modbus/modbus/functions/mbfuncholding_m.o ./Modbus/modbus/functions/mbfuncholding_m.su ./Modbus/modbus/functions/mbfuncinput.cyclo ./Modbus/modbus/functions/mbfuncinput.d ./Modbus/modbus/functions/mbfuncinput.o ./Modbus/modbus/functions/mbfuncinput.su ./Modbus/modbus/functions/mbfuncinput_m.cyclo ./Modbus/modbus/functions/mbfuncinput_m.d ./Modbus/modbus/functions/mbfuncinput_m.o ./Modbus/modbus/functions/mbfuncinput_m.su ./Modbus/modbus/functions/mbfuncother.cyclo ./Modbus/modbus/functions/mbfuncother.d ./Modbus/modbus/functions/mbfuncother.o ./Modbus/modbus/functions/mbfuncother.su ./Modbus/modbus/functions/mbfuncutils.cyclo ./Modbus/modbus/functions/mbfuncutils.d ./Modbus/modbus/functions/mbfuncutils.o ./Modbus/modbus/functions/mbfuncutils.su

.PHONY: clean-Modbus-2f-modbus-2f-functions

