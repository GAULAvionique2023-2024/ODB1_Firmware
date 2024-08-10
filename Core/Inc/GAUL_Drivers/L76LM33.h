/*
 * L76LM33.h
 *
 *  Created on: May 12, 2024
 *      Author: gagnon
 *
 *  Edited on: Jul 4, 2024
 *      Autor: mathouqc
 */

#ifndef INC_GAUL_DRIVERS_L76LM33_H_
#define INC_GAUL_DRIVERS_L76LM33_H_

#include "GAUL_Drivers/NMEA.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"

#define L76LM33_BUFFER_SIZES 256  // NMEA sentence is around 80 char max, has to be a power of two.
#define L76LM33_UART_TIMEOUT 500 // For UART transmit

#define L76LM33_OK 0
#define L76LM33_ERROR -1
#define L76LM33_EMPTY_BUFF -2

typedef struct {
    uint8_t state; // 1: OK, 0: Error with GNSS module
    UART_HandleTypeDef *huart; // Pointer to the GNSS module UART handler
    uint8_t received_byte; // Received char/byte from UART
    ring_buffer_t UART_Buffer; // Ring buffer to store UART data from GNSS module
    char UART_Buffer_arr[L76LM33_BUFFER_SIZES]; // UART buffer array for ring buffer
    uint8_t new_line_flag; // 1: line available in UART buffer, 0: line not available in UART buffer
    char NMEA_Buffer[L76LM33_BUFFER_SIZES]; // Buffer to store NMEA sentence
    GPS_Data gps_data; // Struct to store parsed NMEA data
} L76LM33;

int8_t L76LM33_Init(L76LM33 *L76_data, UART_HandleTypeDef *huart);

void L76LM33_RxCallback(L76LM33 *L76_data, UART_HandleTypeDef *huart);

int8_t L76LM33_Read(L76LM33 *L76_data);
int8_t L76LM33_Read_Sentence(L76LM33 *L76_data);

int8_t L76LM33_Send_Command(L76LM33 *L76_data, char command[], uint8_t size);

#endif /* INC_GAUL_DRIVERS_L76LM33_H_ */
