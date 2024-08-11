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
    // Set Pyro_Test~ (inverse) et Pyros_ON LOW
    Write_GPIO(GPIOA, 15, HIGH);
    Write_GPIO(GPIOB, 4, LOW);
    Write_GPIO(GPIOB, 5, LOW);
}

uint8_t Pyro_Fire(bool armed, char pyro) {

    if (armed == true) {
        if (pyro == 0) {
            Write_GPIO(GPIOB, 4, HIGH);
        } else if (pyro == 1) {
            Write_GPIO(GPIOB, 5, HIGH);
        } else return 0;
    } else return 0;

    HAL_Delay(10);
    Write_GPIO(GPIOA, 15, HIGH);
    Write_GPIO(GPIOB, 4, LOW);
    Write_GPIO(GPIOB, 5, LOW);
    return 1;
}
