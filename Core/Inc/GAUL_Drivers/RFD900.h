/*
 * RFD900.h
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_RFD900_H_
#define INC_GAUL_DRIVERS_RFD900_H_


#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/BMP280.h"
#include "GAUL_Drivers/L76LM33.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"

#include <stdint.h>


typedef struct {
	uint8_t 	mode; 			// 2bits
	uint8_t 	states; 		// 6bits states + mode
	uint8_t		data; 			// Depend du mode
	uint16_t 	validationCRC;	// CRC16 hex
	size_t 		size; 			// depend du mode
} RFD900;

void RFD900_Init(RFD900 *devRFD, uint8_t mode);

uint8_t *RFD900_Create(RFD900 *devRFD, uint8_t mode);
uint8_t RFD900_Send(RFD900 *devRFD);

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
