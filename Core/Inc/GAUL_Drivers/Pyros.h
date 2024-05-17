/*
 * Pyros.h
 *
 *  Created on: May 15, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_PYROS_H_
#define INC_GAUL_DRIVERS_PYROS_H_

#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include <stdbool.h>

bool Pyro_Armed();
void Pyro_Fire(bool armed);


#endif /* INC_GAUL_DRIVERS_PYROS_H_ */
