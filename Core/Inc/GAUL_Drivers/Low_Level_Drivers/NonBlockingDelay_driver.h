/*
 * NonBlockingDelay_driver.h
 *
 *  Created on: Jun 13, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NONBLOCKINGDELAY_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NONBLOCKINGDELAY_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"

bool Delay_Wait(uint32_t delay);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NONBLOCKINGDELAY_DRIVER_H_ */
