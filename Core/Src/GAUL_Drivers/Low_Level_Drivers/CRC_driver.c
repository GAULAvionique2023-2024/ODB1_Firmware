/*
 * CRC_driver.c
 *
 *  Created on: Feb 16, 2024
 *      Author: gagno
 */

#include "main.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"

uint16_t CRC16_Calculate(uint8_t *data, uint8_t size) {

    CRC->CR = CRC_CR_RESET;
    CRC->DR = 0xFFFF;

    uint16_t crc = CRC->DR;

    for (uint8_t i = 0; i < size; ++i) {
        crc ^= (uint16_t)(data[i]) << 8;

        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ POLYNOMIAL_COMPUTATION;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

