/*
 * RFD900.c
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/RFD900.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"

uint8_t RFD900_Init(RFD900 *dev) {


}

void RFD900_DataArray_Update(RFD900 *devRFD, ICM20602 *devICM) {

	// GPS


	// Gyro/Acc
	//uint16_t gyroX = RFD900_ReverseByte16();

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

	// Barometre

}

void RFD900_CreatePacket(RFD900 *dev, uint8_t data[]) {

	// size = sizeof(data) / 8) = 28
	int i = 0;
	while (i < 27)
	{
		dev->packetToSend[27 - i] = data[i]; // packetToSend[29-2] reste 1-0
	}
}

void RFD900_Transmit_RFDTX(RFD900 *dev) {


}

uint8_t RFD900_ReverseByte16(uint16_t value) {

	uint16_t invertedByte = 0;

	int i;
	for(i = 0; i < 16; ++i)
	{
		invertedByte |= ((value >> i) & 1) << (15 - i);
	}

	return invertedByte;
}

