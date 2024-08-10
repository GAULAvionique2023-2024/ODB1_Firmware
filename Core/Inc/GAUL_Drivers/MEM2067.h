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

typedef struct {
	uint32_t total_space;
	uint32_t free_space;
} MEM2067;

uint8_t MEM2067_Mount(const char *filename);
void MEM2067_Write(const char *filename, const char *data);
char *MEM2067_Read(const char *filename);
void MEM2067_Unmount(void);
void MEM2067_Infos(MEM2067 *devMEM);

const char* FATFS_ErrorToString(FRESULT result);

int bufsize(char *buf);
void bufclear(char *p_Buffer);

#endif /* INC_GAUL_DRIVERS_MEM2067_H_ */
