/*
 * RFD900.c
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/RFD900.h"
#include "GAUL_Drivers/CD74HC4051.h"

void RFD900_Init(RFD900 *devRFD, uint8_t mode) {

	/*
	// Gyro/Acc
	devRFD->data[6] = (uint8_t)(devICM->gyroX) >> 8;
	devRFD->data[7] = (uint8_t)(devICM->gyroX) & 0xFF;
	devRFD->data[8] = (uint8_t)(devICM->gyroY) >> 8;
	devRFD->data[9] = (uint8_t)(devICM->gyroY) & 0xFF;
	devRFD->data[10] = (uint8_t)(devICM->gyroZ) >> 8;
	devRFD->data[11] = (uint8_t)(devICM->gyroZ) & 0xFF;

	devRFD->data[12] = (uint8_t)(devICM->accX) >> 8;
	devRFD->data[13] = (uint8_t)(devICM->accX) & 0xFF;
	devRFD->data[14] = (uint8_t)(devICM->accY) >> 8;
	devRFD->data[15] = (uint8_t)(devICM->accY) & 0xFF;
	devRFD->data[16] = (uint8_t)(devICM->accZ) >> 8;
	devRFD->data[17] = (uint8_t)(devICM->accZ) & 0xFF;

	devRFD->data[18] = (uint8_t)(devICM->temperatureC) >> 8;
	devRFD->data[19] = (uint8_t)(devICM->temperatureC) & 0xFF;

	devRFD->data[20] = (uint8_t)(devICM->kalmanAngleRoll) >> 8;
	devRFD->data[21] = (uint8_t)(devICM->kalmanAngleRoll) & 0xFF;
	devRFD->data[22] = (uint8_t)(devICM->kalmanAnglePitch) >> 8;
	devRFD->data[23] = (uint8_t)(devICM->kalmanAnglePitch) & 0xFF;
	// CRC
	devRFD->validationCRC = CRC16_Calculate(devRFD->data, DATA_ARRAY_SIZE);
	*/
}

uint8_t *RFD900_Create(RFD900 *devRFD, uint8_t mode) {
	/*
	devRFD->validationCRC = 0x0000;

	int i = 0;
	while (i < DATA_ARRAY_SIZE)
	{
		devRFD->packetToSend[i] = data[i];
	}

	devRFD->packetToSend[30] = (uint8_t)(devRFD->validationCRC) >> 8;
	devRFD->packetToSend[31] = (uint8_t)(devRFD->validationCRC) & 0xFF;
	*/

	//return packet; // OK
}

uint8_t RFD900_Send(RFD900 *devRFD) {

	//USART_TX(RFD_USART_PORT, devRFD->packetToSend, PACKET_ARRAY_SIZE);

	return 1; // OK
}

