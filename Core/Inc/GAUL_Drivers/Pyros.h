/*
 * Pyros.h
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_PYROS_H_
#define INC_GAUL_DRIVERS_PYROS_H_

#include <GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h>
#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define PYRO_0 0
#define PYRO_1 1

#define PYRO_PIN_0 4
#define PYRO_PIN_1 5

void Pyro_Init(void);
void Pyro_Arming(bool arming);
uint8_t Pyro_Fire(char pyro);

#endif /* INC_GAUL_DRIVERS_PYROS_H_ */
