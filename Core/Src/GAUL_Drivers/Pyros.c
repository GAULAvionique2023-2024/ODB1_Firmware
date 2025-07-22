/*
 * Pyros.c
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Pyros.h"

void Pyro_Init(void) {
    // Pyros
    Init_GPIO(GPIOB, 4, OUT2, O_GP_PP); // PyroON0
    Init_GPIO(GPIOB, 5, OUT2, O_GP_PP); // PyroON1
    Init_GPIO(GPIOA, 15, OUT2, O_GP_PP); // Pyro_Test~
    Init_GPIO(GPIOA, 1, OUT2, O_GP_PP); // Active le 12v
    // Set Pyro_Test~ (inverse) et Pyros_ON LOW
	Write_GPIO(GPIOA, 15, HIGH);
    Write_GPIO(GPIOB, 4, LOW);
    Write_GPIO(GPIOB, 5, LOW);

}

void Pyro_Arming(bool arming) {
	Write_GPIO(GPIOA, 1, arming);
}

uint8_t Pyro_Fire(char pyro) {

	uint8_t target_pin;
	switch (pyro) {
		case PYRO_0: target_pin = PYRO_PIN_0;
			break;
		case PYRO_1: target_pin = PYRO_PIN_1;
			break;
		default:
			return 0;
	}

	Write_GPIO(GPIOB, target_pin, HIGH);
	HAL_Delay(100);		// TODO: Remplacer HAL_Delay par un timer non-bloquant ou une callback
	Write_GPIO(GPIOB, target_pin, LOW);
    return 1;
}
