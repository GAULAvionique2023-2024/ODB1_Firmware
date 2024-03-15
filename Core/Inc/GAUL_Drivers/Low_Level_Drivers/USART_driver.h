/*
 * USART_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_

#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"

#include <stdint.h>

#define RFD_USART_PORT 1
#define GPS_USART_PORT 2
#define BT_USART_PORT  3

void USART_Init(unsigned short usart);

void USART_TX(unsigned short usart, uint8_t *data, int size); // data 8bits
void USART_RX(unsigned short usart, uint8_t *data, int size); // data 8bits

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_ */
