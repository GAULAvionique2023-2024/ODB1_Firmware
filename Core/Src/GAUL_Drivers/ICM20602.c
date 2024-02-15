/*
 * ICM20602.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Luka
 */

#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"

uint8_t ICM20602_Init(ICM20602 *dev)
{
	dev->gyroXRaw = 			0.0f;
	dev->gyroYRaw = 			0.0f;
	dev->gyroZRaw = 			0.0f;

	dev->accXRaw = 				0.0f;
	dev->accYRaw = 				0.0f;
	dev->accZRaw = 				0.0f;

	dev->temperatureC = 		0.0f;

	uint8_t errorCount = 0;
	uint8_t test = 0;
	uint8_t rxData[1];

	// Reset ICM20602
	ICM20602_Write(ICM20602_REG_PWR_MGMT_1, 0x80);
	HAL_Delay(50);

	// Lock SPI communication
	ICM20602_Write(ICM20602_REG_I2C_IF, 0x40);
	HAL_Delay(50);

	// Enable temperature sensor
	ICM20602_Write(ICM20602_REG_PWR_MGMT_1, 0x01);
	HAL_Delay(50);

	// Set sample rate to 1000Hz and apply a software filter
	ICM20602_Write(ICM20602_REG_SMPLRT_DIV, 0x00);
	HAL_Delay(50);

	// Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	ICM20602_Write(ICM20602_REG_CONFIG, 0x05);
	HAL_Delay(50);

	// ACCEL_CONFIG 0x1C
	ICM20602_Write(ICM20602_REG_ACCEL_CONFIG, 0x18); // Acc sensitivity 16g
	HAL_Delay(50);

	// ACCEL_CONFIG2 0x1D
	ICM20602_Write(ICM20602_REG_ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
	HAL_Delay(50);



	ICM20602_Read(ICM20602_REG_WHO_AM_I, rxData, 1);
	if(rxData[0] == 0x12){test = 1;}else{test = 0;}
	printf("ICM20602_REG_WHO_AM_I: %x : %d \n", rxData[0], test);

	ICM20602_Read(ICM20602_REG_I2C_IF, rxData, 1);
	if(rxData[0] == 0x40){test = 1;}else{test = 0;}
	printf("ICM20602_REG_I2C_IF: %x : %d \n", rxData[0], test);

	ICM20602_Read(ICM20602_REG_PWR_MGMT_1, rxData, 1);
	if(rxData[0] == 0x01){test = 1;}else{test = 0;}
	printf("ICM20602_REG_PWR_MGMT_1: %x : %d \n", rxData[0], test);

	ICM20602_Read(ICM20602_REG_SMPLRT_DIV, rxData, 1);
	if(rxData[0] == 0x00){test = 1;}else{test = 0;}
	printf("ICM20602_REG_SMPLRT_DIV: %x : %d \n", rxData[0], test);

	ICM20602_Read(ICM20602_REG_CONFIG, rxData, 1);
	if(rxData[0] == 0x05){test = 1;}else{test = 0;}
	printf("ICM20602_REG_CONFIG: %x : %d \n", rxData[0], test);

	ICM20602_Read(ICM20602_REG_ACCEL_CONFIG, rxData, 1);
	if(rxData[0] == 0x18){test = 1;}else{test = 0;}
	printf("ICM20602_REG_ACCEL_CONFIG: %x : %d \n", rxData[0], test);

	ICM20602_Read(ICM20602_REG_ACCEL_CONFIG2, rxData, 1);
	if(rxData[0] == 0x03){test = 1;}else{test = 0;}
	printf("ICM20602_REG_ACCEL_CONFIG2: %x : %d \n", rxData[0], test);








	return errorCount;
}

void ICM20602_Update_All(ICM20602 *dev)
{
	uint8_t 	rxData[14];

	ICM20602_Read(ICM20602_REG_ACCEL_XOUT_H, rxData, 14);

	dev->accXRaw = ((uint16_t)rxData[0] << 8) | rxData[1];
	dev->accYRaw = ((uint16_t)rxData[2] << 8) | rxData[3];
	dev->accZRaw = ((uint16_t)rxData[4] << 8) | rxData[5];

	dev->temperatureC = (((uint16_t)rxData[6] << 8) | rxData[7])/326.8f + 25;

	dev->gyroXRaw = ((uint16_t)rxData[ 8] << 8) | rxData[ 9];
	dev->gyroYRaw = ((uint16_t)rxData[10] << 8) | rxData[11];
	dev->gyroZRaw = ((uint16_t)rxData[12] << 8) | rxData[13];
}

void ICM20602_Raw_To_Real(ICM20602 *dev)
{
	dev->temperatureC = (dev->temperatureC/326.8) + 25;
}

void ICM20602_Read(uint8_t address, uint8_t rxData[], uint8_t size)
{
	address |= 0x80;  // read operation

	Write_GPIO(PB, 12, LOW);
	SPI2_TX(&address, 1);  // send address
	SPI2_RX(rxData, size);  // receive 6 bytes data
	Write_GPIO(PB, 12, HIGH);
}

void ICM20602_Write(uint8_t address, uint8_t value)
{
	Write_GPIO(PB, 12, LOW);
	SPI2_TX(&address, 1);  // send address
	SPI2_TX(&value, 1);  // send value
	Write_GPIO(PB, 12, HIGH);
}


