/*
 * Pyros.c
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Pyros.h"

void Pyro_Init(void) {
	// Pyros
	Init_GPIO(PB, 4, OUT2, O_GP_PP); // PyroON0
	Init_GPIO(PB, 5, OUT2, O_GP_PP); // PyroON1
	Init_GPIO(PA, 15, OUT2, O_GP_PP); // Pyro_Test~
	// Set Pyro_Test~ (inverse) et Pyros_ON LOW
	Write_GPIO(PA, 15, HIGH);
	Write_GPIO(PB, 4, LOW);
	Write_GPIO(PB, 5, LOW);
}

void Pyro_Fire(bool armed, char pyro) {

	if(armed == true) {
		if(pyro == 0) {
			Write_GPIO(PB, 4, HIGH);
		}
		if(pyro == 1) {
			Write_GPIO(PB, 5, HIGH);
		}
	}
	HAL_Delay(10);
	Write_GPIO(PA, 15, HIGH);
	Write_GPIO(PB, 4, LOW);
	Write_GPIO(PB, 5, LOW);
}
