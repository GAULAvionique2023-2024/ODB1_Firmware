/*
 * SPI_driver.h
 *
 *  Created on: Feb 11, 2024
 *      Author: Luka
 */

#ifndef SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_
#define SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_


void SPI_Init(unsigned short spi);

void SPI1_TX_Char(char tx_char);
void SPI2_TX_Char(char tx_char);

#endif /* SRC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_SPI_DRIVER_H_ */
