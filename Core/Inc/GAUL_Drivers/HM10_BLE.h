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

#define CAM_NUMBER 2

typedef struct {
	bool bluetooth_status;
	bool antenne_status;
	bool gyroscope_status;
	bool accelerometer_status;
	bool gps_status;
	bool barometer_status;
	uint8_t batterie_status; // charge restante (peut-etre)
	bool storage_status;
	// ...
} HM10BLE_Status;

void HM10BLE_Init(void);

void HM10BLE_SendCommand(char *command);
void HM10BLE_Read(char *response);
void HM10BLE_Send(char rx_buffer[], HM10BLE_Status status);


#endif /* INC_GAUL_DRIVERS_HM10_BLE_H_ */
