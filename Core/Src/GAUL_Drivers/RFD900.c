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

	uint8_t *packet;

	mode &= 0x03;
	uint8_t state1; //= CD74HC4051_AnRead(&hacd, CHANNEL_0, PYRO_CHANNEL_0, VREF12) & 0x01;
	uint8_t state2; //= CD74HC4051_AnRead(&hacd, CHANNEL_0, PYRO_CHANNEL_1, VREF12) & 0x01;
	uint8_t state3; //=... & 0x01;
	uint8_t state4; //=... & 0x01;
	uint8_t state5; //=... & 0x01;
	uint8_t state6; //=... & 0x01;

	switch(mode){
		case MODE_PREFLIGHT:
			devRFD->mode = mode;
			devRFD->size = PREFLIGHT_SIZE;
			break;
		case MODE_FLIGHT:
			devRFD->mode = mode;
			devRFD->size = FLIGHT_SIZE;
			break;
		case MODE_POSTFLIGHT:
			devRFD->mode = mode;
			devRFD->size = POSTFLIGHT_SIZE;
			break;
	}

	devRFD->stats = (mode << 6) | (state1 << 5) | (state2 << 4) | (state3 << 3) | (state4 << 2) | (state5 << 1) | state6;
}

void RFD900_Send(RFD900 *devRFD) {

	//USART_TX(RFD_USART_PORT, devRFD->packetToSend, PACKET_ARRAY_SIZE);
}

