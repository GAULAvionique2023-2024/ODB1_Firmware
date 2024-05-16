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

// $Stats(8);V_Batt_Lipo1(16);V_Batt_Lipo2(16);V_Batt_Lipo3 (16);5V_AN (16);Temp (32);Altitude(16);AngleRoll(32);AnglePitch(32)*CRC(16)$ => (28)
#define MODE_PREFLIGHT_SIZE 28 // mode = 0x00
// $Stats(8);
#define MODE_FLIGHT_SIZE // mode = 0x01
#define MODE_POSTFLIGHT_SIZE // mode = 0x02


typedef struct {
	uint8_t 	mode; 			// 2bits
	size_t 		size; 			// depend du mode
	uint8_t 	stats; 			// 6bits states
	uint8_t 	*data; 			// Depend du mode
	uint16_t 	validationCRC;
} RFD900;

uint8_t RFD900_Init(RFD900 *devRFD);

void RFD900_Create(RFD900 *devRFD);
void RFD900_Send(RFD900 *devRFD);

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
