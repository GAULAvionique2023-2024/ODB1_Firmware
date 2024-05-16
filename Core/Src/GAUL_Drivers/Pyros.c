/*
 * Pyros.c
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Pyros.h"

bool Pyro_Armed() {

	// Pyros
	Init_GPIO(PB, 4, OUT2, O_GP_PP); // PyroON0
	Init_GPIO(PB, 5, OUT2, O_GP_PP); // PyroON1
	Init_GPIO(PA, 15, OUT2, O_GP_PP); // Pyro_Test~
	// Set Pyro_Test~ (inverse) et Pyros_ON LOW
	Write_GPIO(PA, 15, HIGH);
	Write_GPIO(PB, 4, LOW);
	Write_GPIO(PB, 5, LOW);

	return true;
}

void Pyro_Fire() {

	if(Pyro_Armed()) {
		Write_GPIO(PA, 15, HIGH);
		Write_GPIO(PB, 4, HIGH);
		Write_GPIO(PB, 5, HIGH);
	}
}
