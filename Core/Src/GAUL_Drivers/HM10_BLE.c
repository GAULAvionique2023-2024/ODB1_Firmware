/*
 * HM10_BLE.c
 *
 *  Created on: May 9, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/HM10_BLE.h"
#include <string.h>
#include <stdbool.h>

const char *valid_commands[] = {
    "TEST",
    "BUZZ",
    "LED",
    "PYRO1",
    "PYRO2",
    "BMP",
    "ICM",
    "GPS",
    "RFD",
    "SD"
};
#define NUM_COMMANDS (sizeof(valid_commands) / sizeof(valid_commands[0]))


uint8_t HM10BLE_Init(HM10BLE *devHM10) {

	devHM10->status = false;

    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+RENEW\r\n"); // Reset all default parameters
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+RESET\r\n"); // Restart to apply changes
    HAL_Delay(500);
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT\r\n"); // Check activity
    // Configuration
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+IMME1\r\n"); // Start
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+NOTI1\r\n"); // Activate notification's module
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+NAMEMerope\r\n"); // Name device
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+ROLE0\r\n"); // Set device in slave mode
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+ADTY0\r\n"); // Set default advertising interval
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT+RESET\r\n"); // Restart to apply changes
    HAL_Delay(500);
    HM10BLE_SendParameters(devHM10, (uint8_t*)"AT\r\n"); // Check activity

    return 1; // OK
}

uint8_t HM10BLE_ConnectionStatus(HM10BLE *devHM10) {

    uint8_t response[128];

    HM10BLE_Read(devHM10, response, sizeof(response));
    if(strstr((char*)response, "OK+CONN")) {
    	devHM10->status = true;
        return 1;
    } else if(strstr((char*)response, "OK+LOST")) {
    	devHM10->status = false;
        return 0;
    } else return 0;
}

bool HM10BLE_ValidCommand(const char *received_command) {

    for(int i = 0; i < NUM_COMMANDS; i++) {
        if(strncmp(received_command, valid_commands[i], strlen(valid_commands[i])) == 0) {
            return true; // Valid
        }
    }
    return false;  // Invalid
}

void HM10BLE_AvailableCommands(HM10BLE *devHM10) {

    char message[256] = "Available commands: ";
    for(int i = 0; i < NUM_COMMANDS; i++) {
        strcat(message, valid_commands[i]);
        if (i < NUM_COMMANDS - 1) {
            strcat(message, ", ");
        }
    }
    strcat(message, "\r\n");
    HM10BLE_Send(devHM10, (uint8_t*)message, strlen(message));
}

void HM10BLE_ExecuteCommand(HM10BLE *devHM10, const char *command) {

    if(strcmp(command, "TEST") == 0) {
        HM10BLE_AvailableCommands(devHM10);
    }
    else if(strcmp(command, "BUZZ") == 0) {
        // test_BUZZ();
    } else if(strcmp(command, "LED") == 0) {
        // test_LED();
    } else if(strcmp(command, "PYRO1") == 0) {
        // test_PYRO1();
    } else if(strcmp(command, "PYRO2") == 0) {
        // test_PYRO2();
    } else if(strcmp(command, "BMP") == 0) {
        // test_BMP();
    } else if(strcmp(command, "ICM") == 0) {
        // test_ICM();
    } else if (strcmp(command, "GPS") == 0) {
        // test_GPS();
    } else if(strcmp(command, "RFD") == 0) {
        // test_RFD();
    } else if(strcmp(command, "SD") == 0) {
        // test_SD();
    } else {
        HM10BLE_Send(devHM10, (uint8_t*)"Command unrecognized\r\n", strlen("Command unrecognized\r\n"));
    }
}

bool HM10BLE_SendParameters(HM10BLE *devHM10, uint8_t *parameter) {

    uint8_t response[100];

    HM10BLE_Send(devHM10, parameter, strlen((char*)parameter));
    HAL_Delay(100); // Delay
    HM10BLE_Read(devHM10, response, sizeof(response));
    return (strcmp((char*)response, "OK\r\n") == 0) ? true : false;
}

uint8_t HM10BLE_CommandTask(HM10BLE *devHM10) {

	if(devHM10->status == false) {
		return 0;
	}

    char command[128];
    memset(command, 0, sizeof(command));
    bool isValid = false;

    HM10BLE_Read(devHM10, (uint8_t*)command, sizeof(command));
    isValid = HM10BLE_ValidCommand(command);
    if(isValid) {
        HM10BLE_ExecuteCommand(devHM10, command);
        return 1; // OK
    }
    return 0; // Failed
}

uint8_t HM10BLE_Read(HM10BLE *devHM10, uint8_t *response, uint8_t size) {

    USART_RX(devHM10->USARTx, response, size);
    return 1; // OK
}

uint8_t HM10BLE_Send(HM10BLE *devHM10, uint8_t *message, uint8_t size) {

    USART_TX(devHM10->USARTx, message, size);
    return 1; // OK
}
