/*
 * structs.h
 *
 *  Created on: Aug 5, 2024
 *      Author: Luka
 */

#ifndef INC_STRUCTS_H_
#define INC_STRUCTS_H_

typedef struct {
    uint8_t mode;
    uint8_t pyro0;
    uint8_t pyro1;
    uint8_t accelerometer;
    uint8_t barometer;
    uint8_t gps;
    uint8_t rfd;
    uint8_t sd;
    uint8_t ble;
} ROCKET_states;

typedef struct {
    ROCKET_states header_states;
    uint8_t *data;
    uint8_t crc16[2];
    uint8_t size;
} ROCKET_Data;

typedef struct {
    uint32_t start_time;
    uint32_t elapsed_time_ms;
    uint8_t elapsed_time_s;
    uint16_t elapsed_time_m;
    uint16_t elapsed_time_remaining_ms;
} RunTimer;

#endif /* INC_STRUCTS_H_ */
