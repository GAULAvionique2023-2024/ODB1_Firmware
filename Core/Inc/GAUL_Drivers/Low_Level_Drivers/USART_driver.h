/*
 * USART_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_

#include "stm32f1xx_hal.h" // USART_TypeDef and int types

void USART_Init(USART_TypeDef *USARTx);

int8_t USART_TX(USART_TypeDef *USARTx, uint8_t *data, uint16_t size);
int8_t USART_RX(USART_TypeDef *USARTx, uint8_t *data, uint16_t size);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_ */
