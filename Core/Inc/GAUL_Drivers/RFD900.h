/*
 * RFD900.h
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_RFD900_H_
#define INC_GAUL_DRIVERS_RFD900_H_

#include "stdint.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/BMP280.h"
//#include "GAUL_Driver/L76LM33"

/*  Structure RFD900 format packet
 *	Array		Type			Size(bits)		Sensor				Interface
 *
	data[0] : 	lattitude 		X			(GPS L76LM33)			USART2
	data[1] : 	longitude 		X			(GPS L76LM33)			USART2
	data[2] : 	altitude 		X			(GPS L76LM33)			USART2

	data[3] : 	gyroX 			16			(Gyro/Acc ICM20602)		SPI1
	data[4] : 	gyroY 			16			(Gyro/Acc ICM20602)		SPI1
	data[5] : 	gyroZ 			16			(Gyro/Acc ICM20602)		SPI1
	data[6] : 	accX 			16			(Gyro/Acc ICM20602)		SPI1
	data[7] : 	accY 			16			(Gyro/Acc ICM20602)		SPI1
	data[8] : 	accZ 			16			(Gyro/Acc ICM20602)		SPI1
	data[9] : 	temperatureC 	16			(Gyro/Acc ICM20602)		SPI1

	data[10] : 	pressure_Pa 	16			(Barometre BMP280)		SPI2
	data[11] : 	temp_C 			16			(Barometre BMP280)		SPI2
 */
//Struct RFD900x
typedef struct {

	//uint8_t time;					// Temps à laquelle l'acquisition des donnees a ete effectuee
	uint8_t data[12];				// Array se trouvant dans un buffer contenant toutes les données des sensors a envoyer
	uint16_t validationCRC;			// Paquet se trouvant dans le buffer du CRC après convertion (16bits)

	uint8_t packetToSend[15]; 		//0-14

} RFD900;

/*
 * Exemple:
 * 1. si 2 data en 16bits et un CRC16, je vais avoir besoin d'un array de dimension 6 en 8bits => uint8_t packetToSend[6]
 * 2. je regarde le buffer de tous les sensors pour obtenir les donnees a envoyer en les ajoutant dans mon array packetToSend[1-4] sinon ne pas envoyer de donnees
 * 3. je calcul le CRC16 sur arrayPacket[1-4] et j'ajoute CRC dans packetToSend[5-6]
 * 4. j'envoie après le tout dans buffer RFD_TX par UART via l'interface USART1 par iteration de 8bits
 */

// Initialisation
uint8_t RFD900_Init(RFD900 *dev);

void RFD900_BufferSensorUpdate(RFD900 *devRFD, ICM20602 *devICM);
uint8_t Reverse_Byte(uint8_t bytes);
void RFD900_CreatePacket(RFD900 *dev, uint8_t data[], uint8_t size);

void RFD900_Transmit_RFD_TX(RFD900 *dev, uint8_t size); // Envoit donnees dans le buffer RFD_TX pour que le modem envoit les donnees par lui-meme

// High Level functions

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
