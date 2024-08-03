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
    bool hm10_status;
    bool rfd_status;
    bool icm_status;
    bool l76lm33_status;
    bool bmp_status;
    uint8_t bat_status; // charge restante (peut-etre)
    bool sd_status;
} HM10BLE;

uint8_t HM10BLE_Init(HM10BLE *devHM10, unsigned short usart_port);
uint8_t HM10BLE_Connection(HM10BLE *devHM10, unsigned short usart_port, uint8_t *rx_buffer);

uint8_t HM10BLE_Read(unsigned short usart_port, uint8_t *response, uint8_t size);
uint8_t HM10BLE_Send(unsigned short usart_port, uint8_t *message, uint8_t size);

#endif /* INC_GAUL_DRIVERS_HM10_BLE_H_ */
