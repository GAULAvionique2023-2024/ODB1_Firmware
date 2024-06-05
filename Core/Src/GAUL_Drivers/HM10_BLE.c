/*
 * HM10_BLE.c
 *
 *  Created on: May 9, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/HM10_BLE.h"


uint8_t HM10BLE_Read(unsigned short usart_port, uint8_t *response, uint8_t size);
uint8_t HM10BLE_Send(unsigned short usart_port, uint8_t *message, uint8_t size);

uint8_t HM10BLE_Init(HM10BLE *status) {

	status->hm10_status = false;
	status->rfd_status = false;
	status->icm_status = false;
	status->l76lm33_status = false;
	status->bmp_status = false;
	status->bat_status = 0x00;
	status->sd_status = false;
	return 1; // OK
}

uint8_t HM10BLE_Connection(HM10BLE *status, unsigned short usart_port, uint8_t *rx_buffer) {

	HM10BLE_Send(usart_port, (uint8_t *)command_at, strlen(command_at));
	HM10BLE_Read(usart_port, rx_buffer, strlen(command_at));
	if(rx_buffer == (uint8_t *)"OK"){
		HM10BLE_Send(usart_port, (uint8_t *)paired_message, strlen(paired_message));
		status->hm10_status = 0x01;
		return 1; // OK
	} else {
		status->hm10_status = false;
		return 0; // No connection
	}
}

uint8_t HM10BLE_Read(unsigned short usart_port, uint8_t *response, uint8_t size) {

	USART_RX(usart_port, response, sizeof(response));
	return 1; // OK
}

uint8_t HM10BLE_Send(unsigned short usart_port, uint8_t *message, uint8_t size) {

	USART_TX(usart_port, message, sizeof(message));
	return 1; // OK
}
