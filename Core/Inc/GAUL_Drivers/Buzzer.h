/*
 * buzzer.h
 *
 *  Created on: 04 Feb, 2024
 *      Author: Nath et Sam
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

//include
#include "stm32f1xx_hal.h"
#include "GAUL_Drivers/Low_Level_Drivers/NonBlockingDelay_driver.h"

//Parametres
typedef struct {

    uint8_t 	nbBips;
    int 		frequencyStart;
    int 		frequencyEnd;
    uint32_t 	delayModulation;
    uint32_t 	delayPause;
} buzzParametres_t;

//Routine
typedef enum {

	STOP,
	START,
	PENDING,
	ARMED,
	CRASH,
} buzzRoutines_t;

// Functions
void Buzz(TIM_HandleTypeDef *htim, uint32_t channel, buzzRoutines_t routine);

#endif /* INC_BUZZER_H_ */
