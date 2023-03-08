################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../modbus/mb.c \
../modbus/mbcrc.c \
../modbus/mbfunccoils.c \
../modbus/mbfuncdiag.c \
../modbus/mbfuncdisc.c \
../modbus/mbfuncholding.c \
../modbus/mbfuncinput.c \
../modbus/mbfuncother.c \
../modbus/mbrtu.c \
../modbus/mbutils.c \
../modbus/portevent.c \
../modbus/portserial.c \
../modbus/porttimer.c 

OBJS += \
./modbus/mb.o \
./modbus/mbcrc.o \
./modbus/mbfunccoils.o \
./modbus/mbfuncdiag.o \
./modbus/mbfuncdisc.o \
./modbus/mbfuncholding.o \
./modbus/mbfuncinput.o \
./modbus/mbfuncother.o \
./modbus/mbrtu.o \
./modbus/mbutils.o \
./modbus/portevent.o \
./modbus/portserial.o \
./modbus/porttimer.o 

C_DEPS += \
./modbus/mb.d \
./modbus/mbcrc.d \
./modbus/mbfunccoils.d \
./modbus/mbfuncdiag.d \
./modbus/mbfuncdisc.d \
./modbus/mbfuncholding.d \
./modbus/mbfuncinput.d \
./modbus/mbfuncother.d \
./modbus/mbrtu.d \
./modbus/mbutils.d \
./modbus/portevent.d \
./modbus/portserial.d \
./modbus/porttimer.d 


# Each subdirectory must supply rules for building sources it contributes
modbus/%.o modbus/%.su: ../modbus/%.c modbus/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-modbus

clean-modbus:
	-$(RM) ./modbus/mb.d ./modbus/mb.o ./modbus/mb.su ./modbus/mbcrc.d ./modbus/mbcrc.o ./modbus/mbcrc.su ./modbus/mbfunccoils.d ./modbus/mbfunccoils.o ./modbus/mbfunccoils.su ./modbus/mbfuncdiag.d ./modbus/mbfuncdiag.o ./modbus/mbfuncdiag.su ./modbus/mbfuncdisc.d ./modbus/mbfuncdisc.o ./modbus/mbfuncdisc.su ./modbus/mbfuncholding.d ./modbus/mbfuncholding.o ./modbus/mbfuncholding.su ./modbus/mbfuncinput.d ./modbus/mbfuncinput.o ./modbus/mbfuncinput.su ./modbus/mbfuncother.d ./modbus/mbfuncother.o ./modbus/mbfuncother.su ./modbus/mbrtu.d ./modbus/mbrtu.o ./modbus/mbrtu.su ./modbus/mbutils.d ./modbus/mbutils.o ./modbus/mbutils.su ./modbus/portevent.d ./modbus/portevent.o ./modbus/portevent.su ./modbus/portserial.d ./modbus/portserial.o ./modbus/portserial.su ./modbus/porttimer.d ./modbus/porttimer.o ./modbus/porttimer.su

.PHONY: clean-modbus

