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

void RFD900_BufferSensorUpdate(RFD900 *devRFD, ICM20602 *devICM) {

	//GPS


	//Gyro/Acc
	devRFD->data[3] = (uint8_t)devICM->gyroX;
	devRFD->data[4] = (uint8_t)devICM->gyroY;
	devRFD->data[5] = (uint8_t)devICM->gyroZ;
	devRFD->data[6] = (uint8_t)devICM->accX;
	devRFD->data[7] = (uint8_t)devICM->accY;
	devRFD->data[8] = (uint8_t)devICM->accZ;
	devRFD->data[9] = (uint8_t)devICM->temperatureC;

	//Barometre

}

void RFD900_CreatePacket(RFD900 *dev, uint8_t data[], uint8_t size) {

	//size = sizeof(data) / 8)
	int i = 0;
	while (i < size)
	{
		dev->packetToSend[14 - i] = Reverse_Byte(data[i]); // packetToSend[14-2] reste 1-0
	}
}

void RFD900_Transmit_RFD_TX(RFD900 *dev, uint8_t size) {


}

uint8_t Reverse_Byte(uint8_t bytes) {

	uint8_t invertedByte = 0;

	int i;
	for(i = 0; i < 8; ++i)
	{
		invertedByte |= ((bytes >> i) & 1) << (7 - i);
	}

	return invertedByte;
}

