################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.c \
../Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.c \
../Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.c \
../Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.c 

OBJS += \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.o \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.o \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.o \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.o 

C_DEPS += \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.d \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.d \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.d \
./Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/GAUL_Drivers/Low_Level_Drivers/%.o Core/Src/GAUL_Drivers/Low_Level_Drivers/%.su Core/Src/GAUL_Drivers/Low_Level_Drivers/%.cyclo: ../Core/Src/GAUL_Drivers/Low_Level_Drivers/%.c Core/Src/GAUL_Drivers/Low_Level_Drivers/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -Og -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-GAUL_Drivers-2f-Low_Level_Drivers

clean-Core-2f-Src-2f-GAUL_Drivers-2f-Low_Level_Drivers:
	-$(RM) ./Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.cyclo ./Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.d ./Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.o ./Core/Src/GAUL_Drivers/Low_Level_Drivers/CRC_driver.su ./Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.cyclo ./Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.d ./Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.o ./Core/Src/GAUL_Drivers/Low_Level_Drivers/GPIO_driver.su ./Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.cyclo ./Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.d ./Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.o ./Core/Src/GAUL_Drivers/Low_Level_Drivers/SPI_driver.su ./Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.cyclo ./Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.d ./Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.o ./Core/Src/GAUL_Drivers/Low_Level_Drivers/USART_driver.su

.PHONY: clean-Core-2f-Src-2f-GAUL_Drivers-2f-Low_Level_Drivers

