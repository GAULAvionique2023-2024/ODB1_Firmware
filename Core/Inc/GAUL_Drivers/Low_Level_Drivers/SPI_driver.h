/*
 * SPI_driver.h
 *
 *  Created on: Feb 11, 2024
 *      Author: Luka
 */

#ifndef SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_
#define SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_

#include "main.h"
#include <stdint.h>

void SPI_Init(SPI_TypeDef *SPIx);
int SPI_TX(SPI_TypeDef *SPIx, uint8_t *data, int size);
int SPI_RX(SPI_TypeDef *SPIx, uint8_t *data, int size);

#endif /* SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_ */
