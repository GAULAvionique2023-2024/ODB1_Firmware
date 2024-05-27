/*
 * HM10_BLE.c
 *
 *  Created on: May 9, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/HM10_BLE.h"

void HM10BLE_Init(void) {


}

void HM10BLE_SendCommand(char *command) {

}

void HM10BLE_Read(char *response) {

	USART_RX(3, (uint8_t*)response, sizeof(response));
	printf("BLE response: %s/n", response);
}

void HM10BLE_Send(char rx_buffer[], HM10BLE status) {

}
