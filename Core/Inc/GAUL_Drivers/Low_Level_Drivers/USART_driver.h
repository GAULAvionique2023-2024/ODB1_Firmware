/*
 * USART_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#include <stdint.h>


#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_

void USART_Init(unsigned short usart);

void USART_TX(unsigned short usart, uint8_t *data, int size); // data 8bits
void USART_RX(unsigned short usart, uint8_t *data, int size); // data 8bits

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_USART_DRIVER_H_ */
