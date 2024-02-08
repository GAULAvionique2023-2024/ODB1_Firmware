/*
 * Copyright (c) 2020, evilwombat
 *
 * Based on principles described by Martin Hubáček in:
 *  http://www.martinhubacek.cz/arm/improved-stm32-ws2812b-library
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once
#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * Structure definition for one channel (one LED strip).
 * Provide a pointer to your pixel data, and a length (in bytes, not in pixels)
 * Setting length=0 disables the channel.
 */
struct led_channel_info {
    const uint8_t *framebuffer;
    int length;
};

//Struct for the LED Status
struct pixel {
    uint8_t g;
    uint8_t r;
    uint8_t b;
};


//Buffer for the LED Status
#define FRAMEBUFFER_SIZE 24

/* We assume Timer2 runs at 72MHz (the maximum). If yours runs at a different rate, set it here. */
#define TIMER2_FREQ_HZ          72000000

/* Calculate TIM2 period, CH1 pulse width, and CH2 pulse width to be 1.25uS, 0.4uS, and 0.8uS */
#define WS2812_TIMER_PERIOD        (((TIMER2_FREQ_HZ / 1000) * 125) / 100000)
#define WS2812_TIMER_PWM_CH1_TIME  (((TIMER2_FREQ_HZ / 1000) *  40) / 100000)
#define WS2812_TIMER_PWM_CH2_TIME  (((TIMER2_FREQ_HZ / 1000) *  80) / 100000)

/* How many channels (strips) of LEDs we want to control. This must not exceed 16. */
#define WS2812_NUM_CHANNELS     1


#define WS2812_CH0_GPIO      2

void WS2812_Init();
void WS2812_Refresh(const struct led_channel_info *channels);
void WS2812_Solid_Colors(struct pixel *framebuffer, const struct led_channel_info *channels, uint8_t r, uint8_t g, uint8_t b);
