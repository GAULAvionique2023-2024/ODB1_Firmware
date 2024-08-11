/*
 * USART_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_

#include "main.h"
#include <stdint.h>

void USART_Init(USART_TypeDef *USARTx);

int USART_TX(USART_TypeDef *USARTx, uint8_t *data, int size);
int USART_RX(USART_TypeDef *USARTx, uint8_t *data, int size);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_ */
