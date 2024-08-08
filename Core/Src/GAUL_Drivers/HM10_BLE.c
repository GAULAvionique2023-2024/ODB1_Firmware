/*
 * HM10_BLE.c
 *
 *  Created on: May 9, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/HM10_BLE.h"
#include <string.h>

uint8_t HM10BLE_SendCommand(HM10BLE *devHM10, uint8_t *command);
uint8_t HM10BLE_Read(HM10BLE *devHM10, uint8_t *response, uint8_t size);
uint8_t HM10BLE_Send(HM10BLE *devHM10, uint8_t *message, uint8_t size);

uint8_t HM10BLE_Init(HM10BLE *devHM10) {

	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+RENEW\r\n");			// Reset all default parameters
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+RESET\r\n");        	// Restart to apply changes
	HAL_Delay(500);
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT\r\n");
    // Configuration
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+IMME1\r\n");        	// Start
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+NOTI1\r\n");        	// Activate notification's module
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+NAMEMerope\r\n"); 	// Name device
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+ROLE0\r\n");       	// Set device in slave mode
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+ADTY0\r\n");        	// Set default advertising interval
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT+RESET\r\n");        	// Restart to apply changes
	HAL_Delay(500);
	HM10BLE_SendCommand(devHM10, (uint8_t*)"AT\r\n");				// Check activity

    // Initialisation des états
    devHM10->hm10_status = false;
    devHM10->rfd_status = false;
    devHM10->icm_status = false;
    devHM10->l76lm33_status = false;
    devHM10->bmp_status = false;
    devHM10->bat_status = 0x00;
    devHM10->sd_status = false;

    return 1; // OK
}

uint8_t HM10BLE_ConnectionStatus(HM10BLE *devHM10) {

    uint8_t response[100];

    HM10BLE_Read(devHM10, response, sizeof(response));
    if (strstr((char*)response, "OK+CONN")) {
        return 1;
    } else if (strstr((char*)response, "OK+LOST")) {
        return 0;
    } else return 0;
}

uint8_t HM10BLE_SendCommand(HM10BLE *devHM10, uint8_t *command) {

    uint8_t response[100];

    HM10BLE_Send(devHM10, command, strlen((char*)command));
    HAL_Delay(100); // Temporisation
    HM10BLE_Read(devHM10, response, sizeof(response));
    return (strcmp((char*)response, "OK\r\n") == 0) ? 1 : 0;
}

uint8_t HM10BLE_Read(HM10BLE *devHM10, uint8_t *response, uint8_t size) {

    USART_RX(devHM10->USARTx, response, size);
    return 1; // OK
}

uint8_t HM10BLE_Send(HM10BLE *devHM10, uint8_t *message, uint8_t size) {

    USART_TX(devHM10->USARTx, message, size);
    return 1; // OK
}
