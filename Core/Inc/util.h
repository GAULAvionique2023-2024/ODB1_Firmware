#ifndef __UTIL_H
#define __UTIL_H

#include <stdio.h>
#include <stdarg.h>
#include <stdio.h>

#include "stm32f1xx_hal.h"
#include "main.h"

#include "GAUL_Drivers/WS2812_led.h"
#include "GAUL_Drivers/BMP280.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"
#include "GAUL_Drivers/Buzzer.h"
#include "GAUL_Drivers/RFD900.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"
#include "GAUL_Drivers/L76LM33.h"
#include "GAUL_Drivers/HM10_BLE.h"
#include "GAUL_Drivers/CD74HC4051.h"
#include "GAUL_Drivers/Pyros.h"
#include "GAUL_Drivers/MEM2067.h"

// TODO: Check data length always good after update
#define MODE_PREFLIGHT 0x00
#define PREFLIGHT_DATASIZE 28
#define MODE_INFLIGHT 0x01
#define INFLIGHT_DATASIZE 62
#define MODE_POSTFLIGHT 0x02
#define POSTFLIGHT_DATASIZE 34
#define MODE_DEBUG 0x03

#define BMP280_BUFFERSIZE 10

#define ACCZ_MASK (1 << 1) // Mach Lock
#define ALTITUDE_MASK  (1 << 6) // Altitude
#define MACHLOCK_MASK (1 << 7) // Mach Lock

void ROCKET_InitRoutine(void);
uint8_t ROCKET_Behavior(void);
uint8_t ROCKET_ModeRoutine(void);
uint8_t ROCKET_SetMode(uint8_t mode);
bool Altitude_Trend(const float newAltitude);
void STM32_u16To8(uint16_t data, ROCKET_Data rocket_data, uint8_t index);
void STM32_i32To8(int32_t data, ROCKET_Data rocket_data, uint8_t index);

char* ROCKET_ModeToString(uint8_t mode);
const char* ROCKET_BehaviorToString(uint8_t behavior);

void RunTimerInit(RunTimer* dev);
void UpdateTime(RunTimer* dev);
int printt(const char *format, ...);
int _write(int le, char *ptr, int len);

#endif /* __UTIL_H */
