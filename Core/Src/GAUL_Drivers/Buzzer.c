/*
 * buzzer.h
 *
 *  Created on: 04 Feb, 2024
 *      Author: Nath et Sam
 */

//include
#include <GAUL_Drivers\Buzzer.h>

static const buzzParametres_t buzzParams[] = {

    {2, 200, 190, 10, 300},    // STOP
    {10, 280, 270, 15, 300},   // START
    {1, 280, 230, 100, 1000},  // PENDING
    {3, 280, 230, 100, 1000},  // ARMED
    {1, 280, 279, 3000, 10}    // CRASH
};

void Buzz(TIM_HandleTypeDef *htim, uint32_t channel, buzzRoutines_t routine){

	HAL_TIM_PWM_Start(htim, channel);

	const buzzParametres_t *params = &buzzParams[routine];

	uint8_t counter = params->nbBips;
	int freq;

	while (counter > 0) {
		if(Delay_Wait(params->delayPause)) {
			for(freq = params->frequencyStart; freq > params->frequencyEnd; freq--)
			{
				if(Delay_Wait(params->delayModulation) == true) {
					__HAL_TIM_SET_AUTORELOAD(htim, freq);
					__HAL_TIM_SET_COMPARE(htim, channel, freq);
					Delay_Wait(params->delayModulation);
				}
			}
			__HAL_TIM_SET_AUTORELOAD(htim, 0);
			__HAL_TIM_SET_COMPARE(htim, channel, 0);
			counter--;
		}
	}

	HAL_TIM_PWM_Stop(htim, channel);
}

