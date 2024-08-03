/*
 * RFD900.h
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_RFD900_H_
#define INC_GAUL_DRIVERS_RFD900_H_

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"

#include <stdint.h>

typedef struct {
    USART_TypeDef *USARTx;
    uint8_t header; 	// mode + states
    uint8_t *data; 		// Depend du mode
    uint8_t *crc;
    uint8_t size; 		// depend du mode
} RFD900;

int RFD900_Init(RFD900 *devRFD);

uint8_t RFD900_Send(RFD900 *devRFD);

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
