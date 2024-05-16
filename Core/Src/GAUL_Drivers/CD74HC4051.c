/*
 * CD74HC4051.c
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/CD74HC4051.h"

void CD74HC4051_Init (ADC_HandleTypeDef *hadc) {

	// Read pin
	Init_GPIO(PA, 0, IN, I_AN); // MUL_AN
	// Batteries
	Init_GPIO(PC, 13, OUT2, O_GP_PP); // MUL_S0
	Init_GPIO(PC, 14, OUT2, O_GP_PP); // MUL_S1
	Init_GPIO(PC, 15, OUT2, O_GP_PP); // MUL_S2
	Init_GPIO(PB, 8, OUT2, O_GP_PP); // MUL_E~
	// Set MUL_E~ (inverse)
	Write_GPIO(PB, 8, HIGH);
	// Pyros
	Init_GPIO(PB, 4, OUT2, O_GP_PP); // PyroON0
	Init_GPIO(PB, 5, OUT2, O_GP_PP); // PyroON1
	Init_GPIO(PA, 15, OUT2, O_GP_PP); // Pyro_Test~
	// Set Pyro_Test~ (inverse) et Pyros_ON LOW
	Write_GPIO(PA, 15, LOW);
	Write_GPIO(PB, 4, LOW);
	Write_GPIO(PB, 5, LOW);

	//ADC calibration
	HAL_ADC_Stop(hadc);
	HAL_ADCEx_Calibration_Start(hadc);
}

uint16_t CD74HC4051_AnRead(ADC_HandleTypeDef *hadc, uint8_t channel, float vref) {

	if(channel == CHANNEL_1 || channel == CHANNEL_7) {
		return 0;
	}

	Write_GPIO(PB, 8, HIGH); // MUL_E~ (inverse)
	Write_GPIO(PA, 15, LOW); // Pyro_Test (inverse)
	Write_GPIO(PB, 4, HIGH); // Pyro_ON0
	Write_GPIO(PB, 5, HIGH); // Pyro_ON1
	// Set channel
	Write_GPIO(PC, 13, (channel & 0x01) ? HIGH : LOW);
	Write_GPIO(PC, 14, (channel & 0x02) ? HIGH : LOW);
	Write_GPIO(PC, 15, (channel & 0x04) ? HIGH : LOW);
	printf("MULS0: %d\n", Read_GPIO(PC, 13));
	printf("MULS1: %d\n", Read_GPIO(PC, 14));
	printf("MULS3: %d\n", Read_GPIO(PC, 15));
	uint32_t adc_value = ADC_Sampling(hadc);
	Write_GPIO(PB, 8, LOW); // MUL_E~ (inverse)
	Write_GPIO(PA, 15, HIGH); // Pyro_Test (inverse)

	return (uint16_t)(adc_value * vref / 4096);
}

uint32_t ADC_Sampling (ADC_HandleTypeDef *hadc) {

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 100); // Timeout peut etre ajuste
	uint32_t adc_value = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	return adc_value;
}
