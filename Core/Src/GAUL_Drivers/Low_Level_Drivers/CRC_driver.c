/*
 * CRC_driver.c
 *
 *  Created on: Feb 16, 2024
 *      Author: gagno
 */

#include "main.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"


uint16_t CRC16_Calculate(uint8_t *data, uint16_t length) {

	CRC->CR = CRC_CR_RESET;
	CRC->DR = 0xFFFF;

	uint16_t crc = CRC->DR;

	for(uint16_t i = 0; i < length; ++i)
	{
		crc ^= (uint16_t)(data[i]) << 8;

		for(uint16_t j = 0; j < 8; ++j)
		{
			if (crc & 0x8000)
			{
				crc = (crc << 1) ^ POLYNOMIAL_COMPUTATION;
			}
			else
			{
				crc <<= 1;
			}
		}
	}

	return crc;
}


// Exemple main.c :
//uint8_t data[] = "Ceci est un exemple de texte";
//uint32_t size = strlen(data);

//hcrc32 hcrc;
//uint32_t hcrc_value;

//hcrc32_Init(&hcrc);

//hcrc_value = hcrc32_Calculation(&hcrc, data, size);

//printf("Le hcrc est : 0x%08X\n", hcrc_value);




