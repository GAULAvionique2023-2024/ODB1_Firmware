/*
 * MEM2067.h
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_MEM2067_H_
#define INC_GAUL_DRIVERS_MEM2067_H_

#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include <ff.h>
#include <stdint.h>

uint8_t MEM2067_Init(void);
uint8_t MEM2067_WriteFATFS(const char *filename, uint8_t *data, uint16_t size);

#endif /* INC_GAUL_DRIVERS_MEM2067_H_ */
