/*
 * ICM20602.h
 *
 *  Created on: Nov 8, 2023
 *      Author: Luka
 */

#ifndef INC_ICM20602_H_
#define INC_ICM20602_H_

//include
#include "stm32f1xx_hal.h"

#define ICM20602_REG_I2C_IF 0x70
#define ICM20602_REG_WHOAMI 0x75

#define ICM20602_VAL_WHOAMI 0x12

//Sensor struct
typedef struct{

	//Spi handle
	SPI_HandleTypeDef 	*SPI_Handle;

	//Chip select
	uint16_t 			CS_Pin;
	GPIO_TypeDef*		GPIO_Port;

	//data
	float 				girX;
	float 				girY;
	float 				girZ;

	float 				accX;
	float 				accY;
	float 				accZ;

}ICM20602;


uint8_t ICM20602_Init(ICM20602 *dev, SPI_HandleTypeDef *SPI_Handle, uint16_t CS_Pin, GPIO_TypeDef* GPIO_Port);

//Low level fonctions
HAL_StatusTypeDef ICM20602_ReadRegister(ICM20602 *dev, uint8_t *data, uint8_t *reg);
HAL_StatusTypeDef ICM20602_ReadRegisters(ICM20602 *dev, uint8_t *data, uint8_t *reg, uint8_t length);
HAL_StatusTypeDef ICM20602_WriteRegister(ICM20602 *dev, uint8_t *data, uint8_t *reg);

#endif /* INC_ICM20602_H_*/
