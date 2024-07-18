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

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128


void USART_Init(unsigned short usart, uint32_t baudrate);

void USART_DMA_TX(unsigned short usart, const uint8_t *data, int size);
void USART_DMA_RX(unsigned short usart, uint8_t *data, int size);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_ */
