/*
 * CD74HC4051.h
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_CD74HC4051_H_
#define INC_GAUL_DRIVERS_CD74HC4051_H_

#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CHANNEL_0 0x00000000
#define CHANNEL_1 0x00000001
#define CHANNEL_2 0x00000010
#define CHANNEL_3 0x00000011
#define CHANNEL_4 0x00000100
#define CHANNEL_5 0x00000101
#define CHANNEL_6 0x00000110
#define CHANNEL_7 0x00000111


void CD74HC4051_Init (ADC_HandleTypeDef *hadc);
uint16_t CD74HC4051_AnRead(ADC_HandleTypeDef *hadc, uint8_t channel, float vref);

// Implicit
uint32_t ADC_Sampling (ADC_HandleTypeDef *hadc);


#endif /* INC_GAUL_DRIVERS_CD74HC4051_H_ */
