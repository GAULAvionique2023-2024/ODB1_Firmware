/*
 * RFD900.h
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_RFD900_H_
#define INC_GAUL_DRIVERS_RFD900_H_

#define DATA_ARRAY_SIZE 28
#define PACKET_ARRAY_SIZE 30

#include "stdint.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/BMP280.h"
//#include "GAUL_Driver/L76LM33"

/*  Structure RFD900 format packet
 *	Array		Type			Size(bits)			Sensor				Interface
 *
	data[0] : 	lattitude 			X			(GPS L76LM33)			USART2
	data[2] : 	longitude 			X			(GPS L76LM33)			USART2
	data[4] : 	altitude 			X			(GPS L76LM33)			USART2

	data[6] : 	gyroX 				16			(Gyro/Acc ICM20602)		SPI1
	data[8] : 	gyroY 				16			(Gyro/Acc ICM20602)		SPI1
	data[10] : 	gyroZ 				16			(Gyro/Acc ICM20602)		SPI1
	data[12] : 	accX 				16			(Gyro/Acc ICM20602)		SPI1
	data[14] : 	accY 				16			(Gyro/Acc ICM20602)		SPI1
	data[16] : 	accZ 				16			(Gyro/Acc ICM20602)		SPI1
	data[18] : 	temperatureC 		16			(Gyro/Acc ICM20602)		SPI1
	data[20] : 	kalmanAngleRoll 	16			(Gyro/Acc ICM20602)		SPI1
	data[22] : 	kalmanAnglePitch 	16			(Gyro/Acc ICM20602)		SPI1

	data[24] : 	pressure_Pa 		X			(Barometre BMP280)		SPI2
	data[26] : 	temp_C 				X			(Barometre BMP280)		SPI2
 */
//Struct RFD900x
typedef struct {

	//uint8_t time;					// Temps à laquelle l'acquisition des donnees a ete effectuee
	uint8_t data[28];				// Array se trouvant dans un buffer contenant toutes les données des sensors a envoyer
	uint16_t validationCRC;			// Paquet se trouvant dans le buffer du CRC après convertion (16bits)

	uint8_t packetToSend[30]; 		//0-29

} RFD900;

/*
 * Exemple:
 * 1. si 2 data en 16bits et un CRC16, je vais avoir besoin d'un array de dimension 6 en 8bits => uint8_t packetToSend[6]
 * 2. je regarde le buffer de tous les sensors pour obtenir les donnees a envoyer en les ajoutant dans mon array packetToSend[1-4] sinon ne pas envoyer de donnees
 * 3. je calcul le CRC16 sur arrayPacket[1-4] et j'ajoute CRC dans packetToSend[5-6]
 * 4. j'envoie après le tout dans buffer RFD_TX par UART via l'interface USART1 par iteration de 8bits
 */

// Taille fixe : size = data[28] et packetToSend[30]

// Initialisation
uint8_t RFD900_Init(RFD900 *dev);

void RFD900_DataArray_Update(RFD900 *devRFD, ICM20602 *devICM);
void RFD900_CreatePacket(RFD900 *devRFD, uint8_t data[]);

void RFD900_Transmit_RFDTX(RFD900 *devRFD); // Envoit donnees dans le buffer RFD_TX pour que le modem envoit les donnees par lui-meme

#endif /* INC_GAUL_DRIVERS_RFD900_H_ */
