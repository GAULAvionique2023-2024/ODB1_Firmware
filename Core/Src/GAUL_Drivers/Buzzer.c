/*
 * buzzer.h
 *
 *  Created on: 04 Feb, 2024
 *      Author: Nath et Sam
 */

//include
#include <GAUL_Drivers\Buzzer.h>
#include "stm32f1xx_ll_tim.h"

static const buzzParametres_t buzzParams[] = {
        { 2, 200, 190, 10, 300 },    // STOP
        { 10, 280, 270, 15, 300 },   // START
        { 1, 280, 230, 100, 1000 },  // PENDING
        { 3, 280, 230, 100, 1000 },  // ARMED
        { 1, 280, 279, 3000, 10 }    // CRASH
};

void Buzz(TIM_TypeDef *TIMx, uint32_t channel, buzzRoutines_t routine) {
    LL_TIM_CC_EnableChannel(TIMx, channel);  // Démarrer PWM

    const buzzParametres_t *params = &buzzParams[routine];
    uint8_t counter = params->nbBips;
    int freq;

    while (counter > 0) {
        if (Delay_Wait(params->delayPause)) {
            for (freq = params->frequencyStart; freq > params->frequencyEnd; freq--) {
                if (Delay_Wait(params->delayModulation) == true) {
                    LL_TIM_SetAutoReload(TIMx, freq);
                    LL_TIM_OC_SetCompareCH1(TIMx, freq);  // Ajuster en fonction du channel
                    Delay_Wait(params->delayModulation);
                }
            }
            LL_TIM_SetAutoReload(TIMx, 0);
            LL_TIM_OC_SetCompareCH1(TIMx, 0);  // Arrêter PWM
            counter--;
        }
    }

    LL_TIM_CC_DisableChannel(TIMx, channel);  // Arrêter PWM
}


