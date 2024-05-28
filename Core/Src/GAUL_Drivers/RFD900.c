/*
 * RFD900.c
 *
 *  Created on: Feb 19, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/RFD900.h"
#include "GAUL_Drivers/CD74HC4051.h"

void RFD900_Init(RFD900 *devRFD, uint8_t mode) {

    devRFD->mode = mode;
    devRFD->states = 0;
    devRFD->data = NULL;
    devRFD->validationCRC = 0;
    devRFD->size = 0;
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

