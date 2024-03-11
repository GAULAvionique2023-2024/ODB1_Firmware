/*
 * RFD900.h
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_RFD900_H_
#define INC_GAUL_DRIVERS_RFD900_H_

#include "stdint.h"

/*  Struct RFD900 format paket
 *
	data[0] : 	lattitude 				(GPS)					USART2
	data[1] : 	longitude 				(GPS)					USART2
	data[2] : 	altitude 				(GPS)					USART2

	data[3] : 	gyroX 					(Gyro/Acc ICM20602)		SPI1
	data[4] : 	gyroY 					(Gyro/Acc ICM20602)		SPI1
	data[5] : 	gyroZ 					(Gyro/Acc ICM20602)		SPI1
	data[7] : 	accX 					(Gyro/Acc ICM20602)		SPI1
	data[8] : 	accY 					(Gyro/Acc ICM20602)		SPI1
	data[9] : 	accZ 					(Gyro/Acc ICM20602)		SPI1
	data[10] : 	temperatureC 			(Gyro/Acc ICM20602)		SPI1

	data[11] : 	pressure_Pa 			(Barometre BMP280)		SPI2
	data[11] : 	temp_C 					(Barometre BMP280)		SPI2
*/
typedef struct {

	uint8_t order;					// Octet qui permet d'identifier l'ordre des bytes dans le paquet de 32bits (8bits)
	uint8_t data[12];				// Paquet se trouvant dans le buffer du USART1 (8bits chaque paquet)
	uint16_t validationCRC;			// Paquet se trouvant dans le buffer du CRC après convertion (16bits)

} RFD900;

// Initialisation
uint8_t RFD900_Init(RFD900 *dev);

void RFD900_Read(uint8_t address, uint8_t txData[], uint8_t size);
void RFD900_Write(uint8_t address, uint8_t txData[], uint8_t size);

// High Level functions

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
