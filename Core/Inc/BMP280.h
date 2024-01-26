/*
 * BMP280.h
 *
 *  Created on: Nov 4, 2023
 *      Author: Luka
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

//include
#include "stm32f1xx_hal.h"

//defines
#define BMP280_DEVICE_ID		0x58
#define BMP280_RESET_WORD		0xB6

#define BMP280_REG_TEMP_XLSB	0xFC
#define BMP280_REG_TEMP_LSB		0xFB
#define BMP280_REG_TEMP_MSB		0xFA
#define BMP280_REG_PRESS_XLSB	0xF9
#define BMP280_REG_PRESS_LSB	0xF8
#define BMP280_REG_PRESS_MSB	0xF7
#define BMP280_REG_CONFIG		0xF5
#define BMP280_REG_CTRL_MEAS	0xF4
#define BMP280_REG_STATUS		0xF3
#define BMP280_REG_RESET		0xE0
#define BMP280_REG_ID			0xD0
#define BMP280_REG_CALIB		0xA1

//Setting ctrl_meas register	(page 25)
//Temperature 	17bit resolution	010
//Pressure 		20bit resolution	101
//Power mode 	normal				11
//01010111 = 0x57
#define BMP280_SETTING_CTRL_MEAS 0x57

//Setting config register		(page 26)
//Stanby time	0.5ms	000
//IIR filter 	4x		010
//Bit 1			N/A		0
//Spi3w			4wire	0
//00001000 = 0x08
#define BMP280_SETTING_CONFIG 0x08

//Sensor struct
typedef struct{

	//Spi handle
	SPI_HandleTypeDef 	*SPI_Handle;

	//Chip select
	uint16_t 			CS_Pin;
	GPIO_TypeDef*		GPIO_Port;

	//data
	float 				pressure_Pa;
	float 				temp_C;

}BMP280;

//Initialisation
uint8_t BMP280_Init(BMP280 *dev, SPI_HandleTypeDef *SPI_Handle, uint16_t CS_Pin, GPIO_TypeDef* GPIO_Port);

//Reading data
HAL_StatusTypeDef BMP280_ReadTemperature(BMP280 *dev);
HAL_StatusTypeDef BMP280_ReadPressure(BMP280 *dev);

//Low level fonctions
HAL_StatusTypeDef BMP280_ReadRegister(BMP280 *dev, uint8_t *data, uint8_t *reg);
HAL_StatusTypeDef BMP280_ReadRegisters(BMP280 *dev, uint8_t *data, uint8_t *reg, uint8_t length);
HAL_StatusTypeDef BMP280_WriteRegister(BMP280 *dev, uint8_t *data, uint8_t *reg);

#endif /* INC_BMP280_H_ */
