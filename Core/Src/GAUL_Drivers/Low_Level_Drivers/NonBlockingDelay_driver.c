/*
 * NonBlockingDelay_driver.c
 *
 *  Created on: Jun 13, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NonBlockingDelay_driver.h"

uint32_t lastGetTick = 0;

bool Delay_Wait(uint32_t delay) {

    if ((HAL_GetTick() - lastGetTick) >= delay) {
        lastGetTick = HAL_GetTick();
        return true;
    } else {
        return false;
    }
}
