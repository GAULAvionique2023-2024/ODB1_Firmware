/*
 * CD74HC4051.h
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_CD74HC4051_H_
#define INC_GAUL_DRIVERS_CD74HC4051_H_

#include <GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h>
#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define CHANNEL_0 0x00	// Pyro_AN
#define CHANNEL_1 0x01	// Batt_Lipo3S2
#define CHANNEL_2 0x02
#define CHANNEL_3 0x03
#define CHANNEL_4 0x04
#define CHANNEL_5 0x05
#define CHANNEL_6 0x06
#define CHANNEL_7 0x07	// 5V_AN -> rfd900

#define PYRO_CHANNEL_DISABLED 0x00
#define PYRO_CHANNEL_0 0x01
#define PYRO_CHANNEL_1 0x02

#define VREFPYRO 3.3
#define VREF5VAN 3.205
#define VREFLIPO1 2.91338
#define VREFLIPO3 2.93817

uint8_t CD74HC4051_Init(ADC_HandleTypeDef *hadc);
uint16_t CD74HC4051_AnRead(ADC_HandleTypeDef *hadc, uint8_t channel, uint8_t pyro_channel, float vref);

#endif /* INC_GAUL_DRIVERS_CD74HC4051_H_ */
