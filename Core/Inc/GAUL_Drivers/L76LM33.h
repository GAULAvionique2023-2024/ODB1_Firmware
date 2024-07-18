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

uint8_t L76LM33_Init(unsigned short usart_port);

uint8_t L76LM33_Read(unsigned short usart_port, char *rx_data, GPS_Data *gps_data);
uint8_t L76LM33_SendCommand(unsigned short usart_port, char *command);

#endif /* INC_GAUL_DRIVERS_L76LM33_H_ */
