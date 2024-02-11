/*
 * ICM20602.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Luka
 */

#include "GAUL_Drivers/ICM20602.h"

unsigned short ICM20602_Init(ICM20602 *dev, unsigned char spi)
{
	dev->SPI = 	spi;

	dev->girX = 		0.0f;
	dev->girY = 		0.0f;
	dev->girZ = 		0.0f;

	dev->accX = 		0.0f;
	dev->accY = 		0.0f;
	dev->accZ = 		0.0f;

	unsigned short errorCount = 0;


	unsigned short regData = 0x40;
	unsigned short address = ICM20602_REG_I2C_IF;


	status = ICM20602_WriteRegister(dev, &regData, &address);
	errorCount += (status != HAL_OK);

	//check device ID = 0x12
	address = ICM20602_REG_WHOAMI;
	status = ICM20602_ReadRegister(dev, &regData, &address);
	errorCount += (status != HAL_OK);

	if(regData != ICM20602_VAL_WHOAMI)
	{
		return 255;
	}

	return errorCount;
}

char ICM20602_Read(ICM20602 *dev)
{

}

