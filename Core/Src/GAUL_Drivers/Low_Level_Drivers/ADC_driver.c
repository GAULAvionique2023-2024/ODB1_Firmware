/*
 * ADC_driver.c
 *
 *  Created on: May 23, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/ADC_driver.h"

uint8_t ADC_Start(ADC_HandleTypeDef *hadc) {

    HAL_ADC_Start(hadc);
    return 1; // OK
}

uint8_t ADC_Calibration(ADC_HandleTypeDef *hadc) {

    HAL_ADCEx_Calibration_Start(hadc);
    return 1; // OK
}

uint8_t ADC_Stop(ADC_HandleTypeDef *hadc) {

    HAL_ADC_Stop(hadc);
    return 1; // OK
}

uint32_t ADC_Sampling(ADC_HandleTypeDef *hadc) {

    if (!(ADC1->CR2 & ADC_CR2_ADON)) {
        return 0xFFFF;
    }
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY); // Timeout peut etre ajuste
    uint32_t adc_value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return adc_value;
}
