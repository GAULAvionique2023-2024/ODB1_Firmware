/*
 * ICM20602.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Luka
 */

#include "math.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/KalmanFilter.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"

uint8_t ICM20602_Init(ICM20602 *dev)
{
	dev->gyroXRaw = 	0.0f;
	dev->gyroYRaw = 	0.0f;
	dev->gyroZRaw = 	0.0f;
	dev->accXRaw = 		0.0f;
	dev->accYRaw = 		0.0f;
	dev->accZRaw = 		0.0f;

	dev->temperatureC = 0.0f;

	Init_GPIO(PA,  10, IN, I_PP); // Init GPIO for the interrupt

	int8_t errorCount = 0;
	int8_t test = 		0;
	uint8_t rxData[1];

	// Reset ICM20602
	ICM20602_Write(ICM20602_REG_PWR_MGMT_1, 0x80);

	// Lock SPI communication
	ICM20602_Write(ICM20602_REG_I2C_IF, 0x40);

	// Enable temperature sensor
	ICM20602_Write(ICM20602_REG_PWR_MGMT_1, 0x01);

	// Set sample rate to 1000Hz and apply a software filter
	ICM20602_Write(ICM20602_REG_SMPLRT_DIV, 0x00);
	//HAL_Delay(50);

	// Gyro LPF fc 20Hz(bit2:0-100) at 1kHz sample rate
	ICM20602_Write(ICM20602_REG_CONFIG, 0x05);

	// Gyro 2000DPS
	ICM20602_Write(ICM20602_REG_GYRO_CONFIG, 0x018);

	// Accel sensitivity 16g
	ICM20602_Write(ICM20602_REG_ACCEL_CONFIG, 0x18);

	// ACCEL_CONFIG2 0x1D
	ICM20602_Write(ICM20602_REG_ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)

	// Enable interrupts
	ICM20602_Write(ICM20602_REG_INT_ENABLE, 0x01);


	ICM20602_Read(ICM20602_REG_WHO_AM_I, rxData, 1);
	if(rxData[0] == 0x12){test = 1;}else{test = 0;}

	ICM20602_Read(ICM20602_REG_I2C_IF, rxData, 1);
	if(rxData[0] == 0x40){test = 1;}else{test = 0;}

	ICM20602_Read(ICM20602_REG_PWR_MGMT_1, rxData, 1);
	if(rxData[0] == 0x01){test = 1;}else{test = 0;}

	ICM20602_Read(ICM20602_REG_SMPLRT_DIV, rxData, 1);
	if(rxData[0] == 0x00){test = 1;}else{test = 0;}

	ICM20602_Read(ICM20602_REG_CONFIG, rxData, 1);
	if(rxData[0] == 0x05){test = 1;}else{test = 0;}

	ICM20602_Read(ICM20602_REG_ACCEL_CONFIG, rxData, 1);
	if(rxData[0] == 0x18){test = 1;}else{test = 0;}

	ICM20602_Read(ICM20602_REG_ACCEL_CONFIG2, rxData, 1);
	if(rxData[0] == 0x03){test = 1;}else{test = 0;}

	ICM20602_Remove_DC_Offset(dev,2);


	return errorCount;
}

void ICM20602_Update_All(ICM20602 *dev)
{
	uint8_t rxData[14];

	ICM20602_Read(ICM20602_REG_ACCEL_XOUT_H, rxData, 14);

	dev->accXRaw = (rxData[0] << 8) | rxData[1];
	dev->accYRaw = (rxData[2] << 8) | rxData[3];
	dev->accZRaw = (rxData[4] << 8) | rxData[5];

	dev->temperatureC = ((rxData[6] << 8) | rxData[7])/326.8f + 25;

	dev->gyroXRaw = (rxData[ 8] << 8) | rxData[ 9];
	dev->gyroYRaw = (rxData[10] << 8) | rxData[11];
	dev->gyroZRaw = (rxData[12] << 8) | rxData[13];

	dev->gyroX = dev->gyroXRaw * 2000.f / 32768.f;
	dev->gyroY = dev->gyroYRaw * 2000.f / 32768.f;
	dev->gyroZ = dev->gyroZRaw * 2000.f / 32768.f;

	dev->accX = dev->accXRaw * 16.f / 32768.f;
	dev->accY = dev->accYRaw * 16.f / 32768.f;
	dev->accZ = dev->accZRaw * 16.f / 32768.f;

	dev->angleRoll = atan(dev->accY/sqrt(dev->accX*dev->accX+dev->accZ*dev->accZ))*1/(3.142/180);
	dev->anglePitch = -atan(dev->accX/sqrt(dev->accY*dev->accY+dev->accZ*dev->accZ))*1/(3.142/180);

	getRollPitch(dev);

}

void ICM20602_Remove_DC_Offset(ICM20602 *dev, uint8_t mean)
{
	int16_t offset[3] = {0,0,0};
	uint8_t rxData[6];
	int8_t i;

	for(i = 0; i < mean; i++)
	{
		ICM20602_Read(ICM20602_REG_GYRO_XOUT_H, rxData, 6);

		offset[0] += (rxData[0] << 8) | rxData[1];
		offset[1] += (rxData[2] << 8) | rxData[3];
		offset[2] += (rxData[4] << 8) | rxData[5];
	}

	offset[0] /= mean;
	offset[1] /= mean;
	offset[2] /= mean;

	ICM20602_Write(ICM20602_REG_XG_OFFS_USRH, (offset[0]*-2)>>8);
	ICM20602_Write(ICM20602_REG_XG_OFFS_USRL, offset[0]*-2);

	ICM20602_Write(ICM20602_REG_YG_OFFS_USRH, (offset[1]*-2)>>8);
	ICM20602_Write(ICM20602_REG_YG_OFFS_USRL, offset[1]*-2);

	ICM20602_Write(ICM20602_REG_ZG_OFFS_USRH, (offset[2]*-2)>>8);
	ICM20602_Write(ICM20602_REG_ZG_OFFS_USRL, offset[2]*-2);
}

int8_t  ICM20602_Data_Ready()
{
	return Read_GPIO(PA, 10);
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
	HAL_Delay(20);
}


