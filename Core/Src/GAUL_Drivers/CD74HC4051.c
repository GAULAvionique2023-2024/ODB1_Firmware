/*
 * CD74HC4051.c
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/CD74HC4051.h"
#include "GAUL_Drivers/Pyros.h"
#include "GAUL_Drivers/Low_Level_Drivers/ADC_driver.h"


uint32_t ADC_Sampling (ADC_HandleTypeDef *hadc);

uint8_t CD74HC4051_Init (ADC_HandleTypeDef *hadc) {

	if (hadc == NULL) {
		return 0; // Error
	}
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
	Pyro_Init();

	//ADC calibration
	ADC_Stop(hadc);
	ADC_Calibration(hadc);

	return 1; // OK
}

uint16_t CD74HC4051_AnRead(ADC_HandleTypeDef *hadc, uint8_t channel, uint8_t pyro_channel, float vref) {

	if (hadc == NULL) {
		return 0;
	}
	if(channel == CHANNEL_1 || channel == CHANNEL_7) {
		return 0;
	}

	ADC_Start(hadc);

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
	Write_GPIO(PB, 8, LOW); // MUL_E~ (inverse)
	// Lecture
	uint32_t adc_value = ADC_Sampling(hadc);
	// Desactiver pyros (ordre important)
	Write_GPIO(PB, 4, LOW); // Pyro_ON0
	Write_GPIO(PB, 5, LOW); // Pyro_ON1
	Write_GPIO(PA, 15, HIGH); // Pyro_Test~

	return (uint16_t)((adc_value * vref / 4096) * 1000);
}
