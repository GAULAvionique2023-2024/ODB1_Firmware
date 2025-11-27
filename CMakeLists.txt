cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

project(ODB1 C ASM)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME})

# --- MCU flags (compile & link)
set(MCU_FLAGS
  -mcpu=cortex-m3
  -mthumb
  -mfloat-abi=soft
)

# --- Compile options
set(COMMON_FLAGS
  -Wall
  -ffunction-sections
  -fdata-sections
  ${MCU_FLAGS}
)
target_compile_options(${PROJECT_NAME} PRIVATE ${COMMON_FLAGS})

set(CMAKE_C_FLAGS_DEBUG   "-Os -g0 -DDEBUG")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")

# --- Link options (PER TARGET) so presets/cache don’t drop them
# Use the script name that your build actually uses: STM32F103XX_FLASH.ld
target_link_options(${PROJECT_NAME} PRIVATE
  ${MCU_FLAGS}
  --specs=nosys.specs          # provide _read/_sbrk/_close stubs
  -T${CMAKE_SOURCE_DIR}/STM32F103XX_FLASH.ld
  -Wl,--gc-sections
  -Wl,--print-memory-usage
)

# If you truly need float printf later, uncomment the next line (adds ~10–20KB):
# target_link_options(${PROJECT_NAME} PRIVATE -u _printf_float)

# --- Sources
target_sources(${PROJECT_NAME} PRIVATE
  Core/Src/main.c
  Core/Src/util.c
  Core/Src/stm32f1xx_it.c
  Core/Src/stm32f1xx_hal_msp.c
  Core/Src/system_stm32f1xx.c
  Core/Startup/startup_stm32f103c8tx.s
  Core/Src/fatfs_sd.c

  FATFS/App/fatfs.c
  FATFS/Target/user_diskio.c

  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_adc_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_cortex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_dma.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_gpio.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_rcc_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_tim_ex.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_hal_uart.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usart.c
  Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c

  Middlewares/Third_Party/FatFs/src/ff.c
  Middlewares/Third_Party/FatFs/src/ff_gen_drv.c
  Middlewares/Third_Party/FatFs/src/diskio.c
  Middlewares/Third_Party/FatFs/src/option/ccsbcs.c

  
  Core/Src/ringbuffer.c
)

# GAUL drivers (glob)
file(GLOB GAUL_SOURCES
  "${CMAKE_SOURCE_DIR}/Core/Src/GAUL_Drivers/*.c"
  "${CMAKE_SOURCE_DIR}/Core/Src/GAUL_Drivers/Low_Level_Drivers/*.c"
  # "${CMAKE_SOURCE_DIR}/Core/Src/GAUL_Drivers/Tests/*.c"
)
target_sources(${PROJECT_NAME} PRIVATE ${GAUL_SOURCES})

# Includes
target_include_directories(${PROJECT_NAME} PRIVATE
  Core/Inc
  Core/Inc/GAUL_Drivers
  Core/Inc/GAUL_Drivers/Low_Level_Drivers
  Core/Inc/GAUL_Drivers/Tests

  FATFS/Target
  FATFS/App

  Drivers/CMSIS/Include
  Drivers/CMSIS/Device/ST/STM32F1xx/Include
  Drivers/STM32F1xx_HAL_Driver/Inc
  Drivers/STM32F1xx_HAL_Driver/Inc/Legacy

  Middlewares/Third_Party/FatFs/src
)

# Defines
target_compile_definitions(${PROJECT_NAME} PRIVATE
  USE_HAL_DRIVER
  STM32F103xB
  USE_FULL_LL_DRIVER
)

# Link libm for pow()
target_link_libraries(${PROJECT_NAME} m)

# Post-build: use real paths of the produced ELF
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}>/$<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}>/$<TARGET_FILE_BASE_NAME:${PROJECT_NAME}>.bin
)
