/*
 * ICM20602.c
 *
 *  Created on: Nov 8, 2023
 *      Author: Luka
 */

#include <GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h>
#include "math.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/KalmanFilter.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"

extern RunTimer run_timer;
KalmanFilter kalmanPitch;
KalmanFilter kalmanRoll;

uint8_t ICM20602_Init(ICM20602 *dev){

    dev->accResult = 0.0f;
    dev->temperatureC = 0.0f;

    uint8_t rxData;
    uint8_t configArray[] = {
    		ICM20602_REG_I2C_IF, 		0x40,
			ICM20602_REG_PWR_MGMT_1, 	0x01,
			ICM20602_REG_SMPLRT_DIV, 	0x00,
			ICM20602_REG_CONFIG, 		0x05,
			ICM20602_REG_GYRO_CONFIG,	0x18,
			ICM20602_REG_ACCEL_CONFIG,	0x18,
			ICM20602_REG_ACCEL_CONFIG2, 0x03,
			ICM20602_REG_INT_PIN_CFG,	0x28,
			ICM20602_REG_INT_ENABLE,	0x01};

    KalmanFilter_Init(&kalmanPitch);
    KalmanFilter_Init(&kalmanRoll);

    Init_GPIO(dev->cs_port, dev->cs_pin, OUT50, O_GP_PP); // CS
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
    Init_GPIO(dev->int_port, dev->int_pin, IN, I_PP); // Init GPIO for the interrupt

    // Réinitialiser ICM20602
    ICM20602_Write(dev, ICM20602_REG_PWR_MGMT_1, 0x80);
    ICM20602_Read(dev, ICM20602_REG_WHO_AM_I, &rxData, 1);
    if (rxData != 0x12) {
        printf("WHO_AM_I check failed: %02X\n", rxData);
        return 1;
    }

    for(uint8_t i = 0; i < sizeof(configArray); i += 2)
    {
        ICM20602_Write(dev, configArray[i], configArray[i+1]);
        ICM20602_Read(dev, configArray[i], &rxData, 1);
        if (rxData != configArray[i+1])
            return 1;
    }

    //ICM20602_Calibrate(dev, ICM20602_GYRO_CALIB_PRECICION);

    return 0;
}

void ICM20602_Update_All(ICM20602 *dev){

	if(!ICM20602_Data_Ready(dev))
		return;

	uint8_t rxData[14];
	int16_t gyroRawX, gyroRawY, gyroRawZ;
	int16_t accRawX, accRawY, accRawZ;
	ICM20602_Read(dev, ICM20602_REG_ACCEL_XOUT_H, rxData, 14);

	// Lire les données brutes
	accRawX = (int16_t)((rxData[0] << 8) | rxData[1]);
	accRawY = (int16_t)((rxData[2] << 8) | rxData[3]);
	accRawZ = (int16_t)((rxData[4] << 8) | rxData[5]);
	dev->temperatureC = ((rxData[6] << 8) | rxData[7]) / 326.8f + 25;
	gyroRawX = (int16_t)((rxData[8] << 8) | rxData[9]);
	gyroRawY = (int16_t)((rxData[10] << 8) | rxData[11]);
	gyroRawZ = (int16_t)((rxData[12] << 8) | rxData[13]);

	// Convertir les valeurs brutes
	dev->gyroX = gyroRawX * 2000.f / 32768.f;
	dev->gyroY = gyroRawY * 2000.f / 32768.f;
	dev->gyroZ = gyroRawZ * 2000.f / 32768.f;

	dev->accX = accRawX * 16.f / 32768.f;
	dev->accY = accRawY * 16.f / 32768.f;
	dev->accZ = accRawZ * 16.f / 32768.f;

	dev->accResult = sqrt(dev->accX * dev->accX + dev->accY * dev->accY + dev->accZ * dev->accZ);

	// Calculer l'angle de pitch et roll à partir des accéléromètres
	dev->angle_pitch_acc = -(atan2(dev->accX, sqrt(dev->accY*dev->accY + dev->accZ*dev->accZ))*180.0)/M_PI;
	dev->angle_roll_acc  = (atan2(dev->accY, dev->accZ)*180.0)/M_PI;

	dev->kalmanPitch = KalmanFilter_Update(&kalmanPitch, dev->angle_pitch_acc, dev->gyroY);
	dev->kalmanRoll = KalmanFilter_Update(&kalmanRoll, dev->angle_roll_acc, dev->gyroX);
}

void ICM20602_Calibrate(ICM20602 *dev, int8_t p_Sense){

    uint8_t rxData[6];
    int16_t gyroRawX, gyroRawY, gyroRawZ;
    int16_t xOffset = 0;
    int16_t yOffset = 0;
    int16_t zOffset = 0;

    do{
        ICM20602_Read(dev, ICM20602_REG_GYRO_XOUT_H, rxData, 6);
        gyroRawX = (int16_t)((rxData[0] << 8) | rxData[1]);
        gyroRawY = (int16_t)((rxData[2] << 8) | rxData[3]);
        gyroRawZ = (int16_t)((rxData[4] << 8) | rxData[5]);

        if (gyroRawX < -p_Sense) xOffset++;
        else if (gyroRawX > p_Sense) xOffset--;

        if (gyroRawY < -p_Sense) yOffset++;
        else if (gyroRawY > p_Sense) yOffset--;

        if (gyroRawZ < -p_Sense) zOffset++;
        else if (gyroRawZ > p_Sense) zOffset--;

		ICM20602_Write(dev, ICM20602_REG_XG_OFFS_TC_H, (xOffset >> 8) & 0xFF);
		ICM20602_Write(dev, ICM20602_REG_XG_OFFS_TC_L, xOffset & 0xFF);

		ICM20602_Write(dev, ICM20602_REG_YG_OFFS_TC_H, (yOffset >> 8) & 0xFF);
		ICM20602_Write(dev, ICM20602_REG_YG_OFFS_TC_L, yOffset & 0xFF);

		ICM20602_Write(dev, ICM20602_REG_ZG_OFFS_TC_H, (zOffset >> 8) & 0xFF);
		ICM20602_Write(dev, ICM20602_REG_ZG_OFFS_TC_L, zOffset & 0xFF);

    } while ((gyroRawX < -p_Sense || gyroRawX > p_Sense) ||
             (gyroRawY < -p_Sense || gyroRawY > p_Sense) ||
             (gyroRawZ < -p_Sense || gyroRawZ > p_Sense));
}

int8_t ICM20602_Data_Ready(ICM20602 *dev){

    return Read_GPIO(dev->int_port, dev->int_pin);
}

void ICM20602_Read(ICM20602 *dev, uint8_t address, uint8_t rxData[], uint8_t size)
{

    address |= 0x80;  // read operation

    Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0){ /* Handle timeout error */
    	Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
    	return;
    }
    if (SPI_RX(dev->SPIx, rxData, size) != 0){/* Handle timeout error */
    	Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
		return;
    }

    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
}

void ICM20602_Write(ICM20602 *dev, uint8_t address, uint8_t value){

	address &= 0x7F;  // Write operation
	Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0) { /* Handle timeout error */ }
    if (SPI_TX(dev->SPIx, &value, 1) != 0) {   /* Handle timeout error */ }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    HAL_Delay(5);
}
