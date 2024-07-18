/*
 * MEM2067.h
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_MEM2067_H_
#define INC_GAUL_DRIVERS_MEM2067_H_

#define BUFFER_SIZE 1024

#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include <fatfs.h>
#include "fatfs_sd.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

uint8_t MEM2067_Mount(char *filename);
void MEM2067_Write(char *filename, char* data);
void MEM2067_Unmount(void);
void MEM2067_Infos(void);

int bufsize (char* buf);
void bufclear(char* p_Buffer);

#endif /* INC_GAUL_DRIVERS_MEM2067_H_ */
