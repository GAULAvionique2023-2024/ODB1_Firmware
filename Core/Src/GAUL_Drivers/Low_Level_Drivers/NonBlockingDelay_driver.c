/*
 * NonBlockingDelay_driver.c
 *
 *  Created on: Jun 13, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NonBlockingDelay_driver.h"


bool Delay_Wait(uint32_t delay) {

	static uint32_t lastGetTick = 0;

	if((HAL_GetTick() - lastGetTick) >= delay) {
		lastGetTick = HAL_GetTick();
		return true;
	} else {
		return false;
	}
}
