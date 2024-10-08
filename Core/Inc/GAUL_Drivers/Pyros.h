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

void Pyro_Init(void);
uint8_t Pyro_Fire(bool armed, char pyro);

#endif /* INC_GAUL_DRIVERS_PYROS_H_ */
