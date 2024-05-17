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

// $Stats(8);V_Batt_Lipo1(16);V_Batt_Lipo2(16);V_Batt_Lipo3(16);5V_AN(16);Temp(32);Altitude(16);AngleRoll(32);AnglePitch(32)*CRC(16)$ => (28)
#define MODE_PREFLIGHT 0x00
#define PREFLIGHT_SIZE 28 // mode = 0x00
// $Stats(8);GPS_Data(416);Gyro_Data(72)*CRC(16)$ => (64)
#define MODE_FLIGHT 0x01
#define FLIGHT_SIZE 64 // mode = 0x01
// $Stats(8);GPS_Data(296);V_Batt_Lipo1(16);V_Batt_Lipo2(16);V_Batt_Lipo3(16);5V_AN(16) => (46)
#define MODE_POSTFLIGHT 0x02
#define POSTFLIGHT_SIZE 46 // mode = 0x02


typedef struct {
	uint8_t 	mode; 			// 2bits
	uint8_t 	stats; 			// 6bits states + mode
	uint8_t		data; 			// Depend du mode
	uint16_t 	validationCRC;
	size_t 		size; 			// depend du mode
} RFD900;

void RFD900_Init(RFD900 *devRFD, uint8_t mode);

uint8_t *RFD900_Create(RFD900 *devRFD, uint8_t mode);
void RFD900_Send(RFD900 *devRFD);

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
