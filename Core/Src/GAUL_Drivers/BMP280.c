/*
 * BMP280.c
 *
 *  Created on: Nov 4, 2023
 *      Author: Luka
 */
/*
#include "GAUL_Drivers/BMP280.h"

//Initialisation
uint8_t BMP280_Init(BMP280 *dev, SPI_HandleTypeDef *SPI_Handle, uint16_t CS_Pin, GPIO_TypeDef* GPIO_Port)
{
	dev->SPI_Handle = 	SPI_Handle;

	dev->GPIO_Port = 	GPIO_Port;
	dev->CS_Pin = 		CS_Pin;

	dev->temp_C = 		0.0f;
	dev->pressure_Pa = 	0.0f;

	uint8_t errorCount = 0;
	HAL_StatusTypeDef status;

	uint8_t regData;
	uint8_t address = BMP280_REG_ID;

	//check device ID = 0x58
	status = BMP280_ReadRegister(dev, &regData, &address);
	errorCount += ( status != HAL_OK);

	if(regData != BMP280_DEVICE_ID)
	{
		return 255;
	}

	//Reset the device
	regData = 0xB6;
	address = BMP280_REG_RESET;

	status = BMP280_WriteRegister(dev, &regData, &address);
	errorCount += ( status != HAL_OK);
	HAL_Delay(2);

	//Setting config register		(page 26)
	//Stanby time	0.5ms	000
	//IIR filter 	4x		010
	//Bit 1			N/A		0
	//Spi3w			4wire	0
	//00001000 = 0x08
	regData = BMP280_SETTING_CONFIG;
	address = BMP280_REG_CONFIG;

	status = BMP280_WriteRegister(dev, &regData, &address);
	errorCount += ( status != HAL_OK);

	status = BMP280_ReadRegister(dev, &regData, &address);
	errorCount += ( status != HAL_OK);
	errorCount += ( regData != BMP280_SETTING_CONFIG);

	//Config the device ctrl meas
	//Temperature 	17bit resolution	010
	//Pressure 		20bit resolution	101
	//Power mode 	normal				11
	//01010111 = 0x57
	regData = BMP280_SETTING_CTRL_MEAS;
	address = BMP280_REG_CTRL_MEAS;

	status = BMP280_WriteRegister(dev, &regData, &address);
	errorCount += ( status != HAL_OK);


	status = BMP280_ReadRegister(dev, &regData, &address);
	errorCount += ( status != HAL_OK);
	errorCount += ( regData != BMP280_SETTING_CTRL_MEAS);


	return errorCount;
}

HAL_StatusTypeDef BMP280_ReadTemperature(BMP280 *dev)
{
	return 1;
}

//Low level fonction
HAL_StatusTypeDef BMP280_ReadRegister(BMP280 *dev, uint8_t *data, uint8_t *reg)
{
	HAL_StatusTypeDef errorCode;

	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_RESET);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, reg, 1, HAL_MAX_DELAY);
	errorCode = HAL_SPI_Receive(dev->SPI_Handle, data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(dev->GPIO_Port, dev->CS_Pin, GPIO_PIN_SET);

	return errorCode;
}

HAL_StatusTypeDef BMP280_ReadRegisters(BMP280 *dev, uint8_t *data, uint8_t *reg, uint8_t length)
{
	HAL_StatusTypeDef errorCode;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, reg, 1, HAL_MAX_DELAY);
	errorCode = HAL_SPI_Receive(dev->SPI_Handle, data, length, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	return errorCode;
}

HAL_StatusTypeDef BMP280_WriteRegister(BMP280 *dev, uint8_t *data, uint8_t *reg)
{
	HAL_StatusTypeDef errorCode;
	*reg = *reg&0x7F; //flip the MSB to 1 to write

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, reg, 1, HAL_MAX_DELAY);
	errorCode = HAL_SPI_Transmit(dev->SPI_Handle, data, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	*reg = *reg|0x80; //Restore the MSB

	return errorCode;
}
*/
