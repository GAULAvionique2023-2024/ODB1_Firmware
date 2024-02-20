################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GAUL_Drivers/BMP280.c \
../Core/Src/GAUL_Drivers/Buzzer.c \
../Core/Src/GAUL_Drivers/ICM20602.c \
../Core/Src/GAUL_Drivers/KalmanFilter.c \
../Core/Src/GAUL_Drivers/WS2812_led.c 

OBJS += \
./Core/Src/GAUL_Drivers/BMP280.o \
./Core/Src/GAUL_Drivers/Buzzer.o \
./Core/Src/GAUL_Drivers/ICM20602.o \
./Core/Src/GAUL_Drivers/KalmanFilter.o \
./Core/Src/GAUL_Drivers/WS2812_led.o 

C_DEPS += \
./Core/Src/GAUL_Drivers/BMP280.d \
./Core/Src/GAUL_Drivers/Buzzer.d \
./Core/Src/GAUL_Drivers/ICM20602.d \
./Core/Src/GAUL_Drivers/KalmanFilter.d \
./Core/Src/GAUL_Drivers/WS2812_led.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/GAUL_Drivers/%.o Core/Src/GAUL_Drivers/%.su Core/Src/GAUL_Drivers/%.cyclo: ../Core/Src/GAUL_Drivers/%.c Core/Src/GAUL_Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-GAUL_Drivers

clean-Core-2f-Src-2f-GAUL_Drivers:
	-$(RM) ./Core/Src/GAUL_Drivers/BMP280.cyclo ./Core/Src/GAUL_Drivers/BMP280.d ./Core/Src/GAUL_Drivers/BMP280.o ./Core/Src/GAUL_Drivers/BMP280.su ./Core/Src/GAUL_Drivers/Buzzer.cyclo ./Core/Src/GAUL_Drivers/Buzzer.d ./Core/Src/GAUL_Drivers/Buzzer.o ./Core/Src/GAUL_Drivers/Buzzer.su ./Core/Src/GAUL_Drivers/ICM20602.cyclo ./Core/Src/GAUL_Drivers/ICM20602.d ./Core/Src/GAUL_Drivers/ICM20602.o ./Core/Src/GAUL_Drivers/ICM20602.su ./Core/Src/GAUL_Drivers/KalmanFilter.cyclo ./Core/Src/GAUL_Drivers/KalmanFilter.d ./Core/Src/GAUL_Drivers/KalmanFilter.o ./Core/Src/GAUL_Drivers/KalmanFilter.su ./Core/Src/GAUL_Drivers/WS2812_led.cyclo ./Core/Src/GAUL_Drivers/WS2812_led.d ./Core/Src/GAUL_Drivers/WS2812_led.o ./Core/Src/GAUL_Drivers/WS2812_led.su

.PHONY: clean-Core-2f-Src-2f-GAUL_Drivers

