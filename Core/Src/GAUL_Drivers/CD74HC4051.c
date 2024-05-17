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

uint16_t CD74HC4051_AnRead(ADC_HandleTypeDef *hadc, uint8_t channel, uint8_t pyro_channel, float vref) {

	if(channel == CHANNEL_1 || channel == CHANNEL_7) {
		return 0;
	}

	Write_GPIO(PB, 8, HIGH); // MUL_E~ (inverse)
	Write_GPIO(PA, 15, LOW); // Pyro_Test (inverse)
	if(channel == CHANNEL_0) {
		if(pyro_channel == PYRO_CHANNEL_0) {
			Write_GPIO(PB, 4, HIGH); // Pyro_ON0
		} else if (pyro_channel == PYRO_CHANNEL_1) {
			Write_GPIO(PB, 5, HIGH); // Pyro_ON1
		} else {
			return 0;
		}
	} else {
		// Set channel
		Write_GPIO(PC, 13, (channel & 0x01) ? HIGH : LOW);
		Write_GPIO(PC, 14, (channel & 0x02) ? HIGH : LOW);
		Write_GPIO(PC, 15, (channel & 0x04) ? HIGH : LOW);
	}
	// Reactiver multiplexer pour lecture
	printf("Pyro_Test avant: %i\n", Read_GPIO(PA, 15));
	printf("Pyro_ON0 avant: %i\n", Read_GPIO(PB, 4));
	printf("Pyro_ON1 avant: %i\n", Read_GPIO(PB, 5));
	Write_GPIO(PB, 8, LOW); // MUL_E~ (inverse)
	// Lecture
	uint32_t adc_value = ADC_Sampling(hadc);
	// Desactiver pyros (ordre important)
	Write_GPIO(PB, 4, LOW); // Pyro_ON0
	Write_GPIO(PB, 5, LOW); // Pyro_ON1
	Write_GPIO(PA, 15, HIGH); // Pyro_Test~
	printf("Pyro_Test apres: %i\n", Read_GPIO(PA, 15));
	printf("Pyro_ON0 apres: %i\n", Read_GPIO(PB, 4));
	printf("Pyro_ON1 apres: %i\n", Read_GPIO(PB, 5));

	return (uint16_t)((adc_value * vref / 4096) * 1000); // millivolts
}

uint32_t ADC_Sampling (ADC_HandleTypeDef *hadc) {

	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY); // Timeout peut etre ajuste
	uint32_t adc_value = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop(hadc);

	return adc_value;
}
