/*
 * HM10_BLE.h
 *
 *  Created on: May 9, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_HM10_BLE_H_
#define INC_GAUL_DRIVERS_HM10_BLE_H_

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>


typedef struct {
	USART_TypeDef *USARTx;
	bool status;
} HM10BLE;

uint8_t HM10BLE_Init(HM10BLE *devHM10);
uint8_t HM10BLE_ConnectionStatus(HM10BLE *devHM10);
bool HM10BLE_SendParameters(HM10BLE *devHM10, uint8_t *parameter);
uint8_t HM10BLE_CommandTask(HM10BLE *devHM10);

uint8_t HM10BLE_Read(HM10BLE *devHM10, uint8_t *response, uint8_t size);
uint8_t HM10BLE_Send(HM10BLE *devHM10, uint8_t *message, uint8_t size);

#endif /* INC_GAUL_DRIVERS_HM10_BLE_H_ */
