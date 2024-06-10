/*
 * HM10_BLE.c
 *
 *  Created on: May 9, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/HM10_BLE.h"


uint8_t HM10BLE_Read(unsigned short usart_port, uint8_t *response, uint8_t size);
uint8_t HM10BLE_Send(unsigned short usart_port, uint8_t *message, uint8_t size);

uint8_t HM10BLE_Init(HM10BLE *devHM10, unsigned short usart_port) {

	// Configuration
	HM10BLE_Send(usart_port, (uint8_t *)"AT\n", strlen("AT\n"));							// Awake
	HM10BLE_Send(usart_port, (uint8_t *)"AT+NAMEBulldogs\n", strlen("AT+NAMEMerope\n"));	// Name device
	HM10BLE_Send(usart_port, (uint8_t *)"AT+IMME1\n", strlen("AT+IMME1\n"));				// Start

	devHM10->hm10_status = false;
	devHM10->rfd_status = false;
	devHM10->icm_status = false;
	devHM10->l76lm33_status = false;
	devHM10->bmp_status = false;
	devHM10->bat_status = 0x00;
	devHM10->sd_status = false;
	return 1; // OK
}

uint8_t HM10BLE_Connection(HM10BLE *devHM10, unsigned short usart_port, uint8_t *rx_buffer) {

	HM10BLE_Send(usart_port, (uint8_t *)"AT\n", strlen("AT\n"));
	HM10BLE_Read(usart_port, rx_buffer, strlen("AT\n"));
	if(rx_buffer == (uint8_t *)"OK"){
		HM10BLE_Send(usart_port, (uint8_t *)"(+) HM10BLE's STM32 connected...\r\n", strlen("(+) HM10BLE's STM32 connected...\r\n"));
		devHM10->hm10_status = true;
		return 1; // OK
	} else {
		devHM10->hm10_status = false;
		return 0; // No connection
	}
}

uint8_t HM10BLE_Disconnection(HM10BLE *devHM10, unsigned short usart_port, uint8_t *rx_buffer) {


	return 1; // OK
}

uint8_t HM10BLE_Read(unsigned short usart_port, uint8_t *response, uint8_t size) {

	USART_RX(usart_port, response, sizeof(response));
	return 1; // OK
}

uint8_t HM10BLE_Send(unsigned short usart_port, uint8_t *message, uint8_t size) {

	USART_TX(usart_port, message, sizeof(message));
	return 1; // OK
}
