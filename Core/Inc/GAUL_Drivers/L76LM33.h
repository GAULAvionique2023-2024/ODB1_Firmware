/*
 * L76LM33.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_L76LM33_H_
#define INC_GAUL_DRIVERS_L76LM33_H_

#include "GAUL_Drivers/NMEA.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include <string.h>
#include <stdint.h>

#define BUFFER_SIZE 512

typedef struct {
    USART_TypeDef *USARTx;
} L76LM33;

uint8_t L76LM33_Init(L76LM33 *devL76L);

uint8_t L76LM33_Read(L76LM33 *devL76L, char *rx_data, GPS_Data *gps_data);
uint8_t L76LM33_SendCommand(L76LM33 *devL76L, char *command);

#endif /* INC_GAUL_DRIVERS_L76LM33_H_ */
