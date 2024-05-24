/*
 * SPI_driver.h
 *
 *  Created on: Feb 11, 2024
 *      Author: Luka
 */

#ifndef SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_
#define SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_

#include <stdint.h>

void SPI_Init(unsigned short spi);

void SPI1_TX(uint8_t *data, int size);
void SPI2_TX(uint8_t *data, int size);

void SPI1_RX(uint8_t *data, int size);
void SPI2_RX(uint8_t *data, int size);

#endif /* SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_ */
