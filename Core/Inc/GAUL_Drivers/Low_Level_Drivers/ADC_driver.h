/*
 * ADC_driver.h
 *
 *  Created on: May 23, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_ADC_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_ADC_DRIVER_H_

#include "stm32f1xx_hal.h"

uint8_t ADC_Start(ADC_HandleTypeDef *hadc);
uint8_t ADC_Stop(ADC_HandleTypeDef *hadc);
uint8_t ADC_Calibration(ADC_HandleTypeDef *hadc);

uint32_t ADC_Sampling (ADC_HandleTypeDef *hadc);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_ADC_DRIVER_H_ */
