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
#include <GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h>
#include <fatfs.h>
#include "fatfs_sd.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HEADER_NUM 15

// Union pour différents types de données
typedef union {
    int i;
    float f;
    double d;
    short s;
    char c;
    char* str;
} DataUnion;

// Type enum pour les types de données
typedef enum {
    DATA_TYPE_INT,
    DATA_TYPE_FLOAT,
    DATA_TYPE_DOUBLE,
    DATA_TYPE_SHORT,
    DATA_TYPE_CHAR,
	DATA_TYPE_STRING
} DataType;

// Structure pour représenter un champ de données
typedef struct {
    DataType type;
    DataUnion data;
} DataField;

typedef struct {
	uint32_t total_space;
	uint32_t free_space;
} MEM2067;

uint8_t MEM2067_Mount(const char *filename);
void MEM2067_Write(const char *filename, const DataField data[], size_t num_fields);
char *MEM2067_Read(const char *filename);
void MEM2067_Unmount(void);
void MEM2067_Infos(MEM2067 *devMEM);

const char* FATFS_ErrorToString(FRESULT result);

int bufsize(char *buf);
void bufclear(char *p_Buffer);

#endif /* INC_GAUL_DRIVERS_MEM2067_H_ */
