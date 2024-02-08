/*
 * ICM20602.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Luka
 */

#include "GAUL_Drivers/ICM20602.h"

uint8_t ICM20602_Init(ICM20602 *dev, SPI_HandleTypeDef *SPI_Handle, uint16_t CS_Pin, GPIO_TypeDef* GPIO_Port)
{
	dev->SPI_Handle = 	SPI_Handle;

	dev->GPIO_Port = 	GPIO_Port;
	dev->CS_Pin = 		CS_Pin;

	dev->girX = 		0.0f;
	dev->girY = 		0.0f;
	dev->girZ = 		0.0f;

	dev->accX = 		0.0f;
	dev->accY = 		0.0f;
	dev->accZ = 		0.0f;

	uint8_t errorCount = 0;
	HAL_StatusTypeDef status;


	uint8_t regData = 0x40;
	uint8_t address = ICM20602_REG_I2C_IF;


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

//Low level fonction
HAL_StatusTypeDef ICM20602_ReadRegister(ICM20602 *dev, uint8_t *data, uint8_t *reg)
{
	HAL_StatusTypeDef errorCode;

	*reg = *reg|0x80; //flip the MSB to 1 to read
	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_RESET);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, reg, 1, HAL_MAX_DELAY);
	errorCode = HAL_SPI_Receive(dev->SPI_Handle, data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_SET);
	*reg = *reg&0x7F; //Restore the MSB

	return errorCode;
}

HAL_StatusTypeDef ICM20602_ReadRegisters(ICM20602 *dev, uint8_t *data, uint8_t *reg, uint8_t length)
{
	HAL_StatusTypeDef errorCode;

	*reg = *reg|0x80; //flip the MSB to 1 to read
	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_RESET);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, reg, 1, HAL_MAX_DELAY);
	errorCode = HAL_SPI_Receive(dev->SPI_Handle, data, length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_SET);
	*reg = *reg&0x7F; //Restore the MSB

	return errorCode;
}

HAL_StatusTypeDef ICM20602_WriteRegister(ICM20602 *dev, uint8_t *data, uint8_t *reg)
{
	HAL_StatusTypeDef errorCode;

	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_RESET);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, reg, 1, HAL_MAX_DELAY);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_SET);

	return errorCode;
}

