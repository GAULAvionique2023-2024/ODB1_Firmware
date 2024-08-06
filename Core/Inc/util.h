#ifndef __UTIL_H
#define __UTIL_H

#include "stm32f1xx_hal.h"
#include <stdio.h>

typedef struct {
    uint32_t start_time;
    uint32_t elapsed_time_ms;
    uint8_t elapsed_time_s;
    uint16_t elapsed_time_m;
    uint16_t elapsed_time_remaining_ms;
} RunTimer;

int _write(int le, char *ptr, int len);
void RunTimerInit(RunTimer* dev);
void UpdateTime(RunTimer* dev);

#endif /* __UTIL_H */
