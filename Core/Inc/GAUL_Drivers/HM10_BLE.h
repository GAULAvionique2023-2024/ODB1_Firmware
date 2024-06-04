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


typedef struct {
	bool hm10_status;
	bool rfd_status;
	bool icm_status;
	bool l76lm33_status;
	bool bmp_status;
	uint8_t bat_status; // charge restante (peut-etre)
	bool sd_status;
} HM10BLE;

void HM10BLE_Init(void);

void HM10BLE_SendCommand(char *command);
void HM10BLE_Read(unsigned short usart_port,char *response);
void HM10BLE_Send(unsigned short usart_port, char *rx_buffer, HM10BLE *status);
// TODO: add temp_ref + press_ref modification via ble


#endif /* INC_GAUL_DRIVERS_HM10_BLE_H_ */
