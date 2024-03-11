/*
 * CRC_driver.c
 *
 *  Created on: Feb 16, 2024
 *      Author: gagno
 */

#include "main.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"



uint32_t CRC32_Calculate(CRC_HandleTypeDef *hcrc, uint32_t inputBuffer[], uint32_t length) {

	CRC->CR = CRC_CR_RESET;
	CRC->DR = 0xFFFFFFFF;

	return HAL_CRC_Calculate(hcrc, inputBuffer, length);
}


// Exemple main.c :
//uint8_t data[] = "Ceci est un exemple de texte";
//uint32_t size = strlen(data);

//hcrc32 hcrc;
//uint32_t hcrc_value;

//hcrc32_Init(&hcrc);

//hcrc_value = hcrc32_Calculation(&hcrc, data, size);

//printf("Le hcrc est : 0x%08X\n", hcrc_value);




