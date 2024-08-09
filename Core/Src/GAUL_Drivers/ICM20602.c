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

extern RunTimer run_timer;
KalmanFilter kalmanPitch;
KalmanFilter kalmanRoll;

uint8_t ICM20602_Init(ICM20602 *dev)
{
    dev->gyroXRaw = 0;
    dev->gyroYRaw = 0;
    dev->gyroZRaw = 0;
    dev->accXRaw = 0;
    dev->accYRaw = 0;
    dev->accZRaw = 0;
    dev->accResult = 0.0f;
    dev->temperatureC = 0.0f;

    KalmanFilter_Init(&kalmanPitch);
    KalmanFilter_Init(&kalmanRoll);

    Init_GPIO(dev->cs_port, dev->cs_pin, OUT50, O_GP_PP); // CS
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    Init_GPIO(dev->int_port, dev->int_pin, IN, I_PP); // Init GPIO for the interrupt

    uint8_t rxData;

    // Réinitialiser ICM20602
    ICM20602_Write(dev, ICM20602_REG_PWR_MGMT_1, 0x80);
    ICM20602_Read(dev, ICM20602_REG_WHO_AM_I, &rxData, 1);
    if (rxData != 0x12) {
        printf("WHO_AM_I check failed: %02X\n", rxData);
        return 1;
    }

    // Verrouiller la communication SPI
    ICM20602_Write(dev, ICM20602_REG_I2C_IF, 0x40);
    ICM20602_Read(dev, ICM20602_REG_I2C_IF, &rxData, 1);
    if (rxData != 0x40) {
        printf("I2C_IF check failed: %02X\n", rxData);
        return 1;
    }

    // Activer le capteur de température
    ICM20602_Write(dev, ICM20602_REG_PWR_MGMT_1, 0x01);
    ICM20602_Read(dev, ICM20602_REG_PWR_MGMT_1, &rxData, 1);
    if (rxData != 0x01) {
        printf("PWR_MGMT_1 check failed: %02X\n", rxData);
        return 1;
    }

    // Définir la fréquence d'échantillonnage à 1000Hz et appliquer un filtre logiciel
    ICM20602_Write(dev, ICM20602_REG_SMPLRT_DIV, 0x00);
    ICM20602_Read(dev, ICM20602_REG_SMPLRT_DIV, &rxData, 1);
    if (rxData != 0x00) {
        printf("SMPLRT_DIV check failed: %02X\n", rxData);
        return 1;
    }

    // Gyro LPF fc 20Hz(bit2:0-100) à un taux d'échantillonnage de 1kHz
    ICM20602_Write(dev, ICM20602_REG_CONFIG, 0x05);
    ICM20602_Read(dev, ICM20602_REG_CONFIG, &rxData, 1);
    if (rxData != 0x05) {
        printf("CONFIG check failed: %02X\n", rxData);
        return 1;
    }

    // Gyro 2000DPS
    ICM20602_Write(dev, ICM20602_REG_GYRO_CONFIG, 0x18);
    ICM20602_Read(dev, ICM20602_REG_GYRO_CONFIG, &rxData, 1);
    if (rxData != 0x18) {
        printf("GYRO_CONFIG check failed: %02X\n", rxData);
        return 1;
    }

    // Sensibilité de l'accéléromètre 16g
    ICM20602_Write(dev, ICM20602_REG_ACCEL_CONFIG, 0x18);
    ICM20602_Read(dev, ICM20602_REG_ACCEL_CONFIG, &rxData, 1);
    if (rxData != 0x18) {
        printf("ACCEL_CONFIG check failed: %02X\n", rxData);
        return 1;
    }

    // ACCEL_CONFIG2 0x1D
    ICM20602_Write(dev, ICM20602_REG_ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
    ICM20602_Read(dev, ICM20602_REG_ACCEL_CONFIG2, &rxData, 1);
    if (rxData != 0x03) {
        printf("ACCEL_CONFIG2 check failed: %02X\n", rxData);
        return 1;
    }

    // Config INT PIN
    ICM20602_Write(dev, ICM20602_REG_INT_PIN_CFG, 0x28); // Active HIGH, Push-Pull, LATCH, Any read clear
    ICM20602_Read(dev, ICM20602_REG_INT_PIN_CFG, &rxData, 1);
    if (rxData != 0x28) {
        printf("INT_PIN_CFG check failed: %02X\n", rxData);
        return 1;
    }

    // Activer les interruptions
    ICM20602_Write(dev, ICM20602_REG_INT_ENABLE, 0x01);
    ICM20602_Read(dev, ICM20602_REG_INT_ENABLE, &rxData, 1);
    if (rxData != 0x01) {
        printf("INT_ENABLE check failed: %02X\n", rxData);
        return 1;
    }

    ICM20602_Calibrate(dev, ICM20602_GYRO_CALIB_PRECICION);

    return 0;
}

void ICM20602_Update_All(ICM20602 *dev)
{
	if(!ICM20602_Data_Ready(dev))
		return;

	uint8_t rxData[14];
  ICM20602_Read(dev, ICM20602_REG_ACCEL_XOUT_H, rxData, 14);

  // Lire les données brutes et appliquer les offsets
  dev->accXRaw = (int16_t)((rxData[0] << 8) | rxData[1]);
  dev->accYRaw = (int16_t)((rxData[2] << 8) | rxData[3]);
  dev->accZRaw = (int16_t)((rxData[4] << 8) | rxData[5]);
  dev->temperatureC = ((rxData[6] << 8) | rxData[7]) / 326.8f + 25;
  dev->gyroXRaw = (int16_t)((rxData[8] << 8) | rxData[9]);
  dev->gyroYRaw = (int16_t)((rxData[10] << 8) | rxData[11]);
  dev->gyroZRaw = (int16_t)((rxData[12] << 8) | rxData[13]);

  // Convertir les valeurs brutes
  dev->gyroX = dev->gyroXRaw * 2000.f / 32768.f;
  dev->gyroY = dev->gyroYRaw * 2000.f / 32768.f;
  dev->gyroZ = dev->gyroZRaw * 2000.f / 32768.f;

  dev->accX = dev->accXRaw * 16.f / 32768.f;
  dev->accY = dev->accYRaw * 16.f / 32768.f;
  dev->accZ = dev->accZRaw * 16.f / 32768.f;

  dev->accResult = sqrt(dev->accX * dev->accX + dev->accY * dev->accY + dev->accZ * dev->accZ);

  // Calculer l'angle de pitch et roll à partir des accéléromètres
  dev->angle_pitch_acc = -(atan2(dev->accX, sqrt(dev->accY*dev->accY + dev->accZ*dev->accZ))*180.0)/M_PI;
	dev->angle_roll_acc  = (atan2(dev->accY, dev->accZ)*180.0)/M_PI;

	dev->kalmanPitch = KalmanFilter_Update(&kalmanPitch, dev->angle_pitch_acc, dev->gyroY);
	dev->kalmanRoll = KalmanFilter_Update(&kalmanRoll, dev->angle_roll_acc, dev->gyroX);
}

void ICM20602_Calibrate(ICM20602 *dev, int8_t p_Sense)
{
    uint8_t rxData[6];
    int16_t gyroRawX, gyroRawY, gyroRawZ;
    int16_t xOffset = 0;
    int16_t yOffset = 0;
    int16_t zOffset = 0;

    do
    {
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

int8_t ICM20602_Data_Ready(ICM20602 *dev)
{
    return Read_GPIO(dev->int_port, dev->int_pin);
}

void ICM20602_Read(ICM20602 *dev, uint8_t address, uint8_t rxData[], uint8_t size)
{
    address |= 0x80;  // read operation

    Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0)
    { /* Handle timeout error */
    	Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
    	return;
    }
    if (SPI_RX(dev->SPIx, rxData, size) != 0)
    {/* Handle timeout error */
    	Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
		return;
    }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
}

void ICM20602_Write(ICM20602 *dev, uint8_t address, uint8_t value)
{

  address &= 0x7F;  // Write operation


	Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0) { /* Handle timeout error */ }
    if (SPI_TX(dev->SPIx, &value, 1) != 0) {   /* Handle timeout error */ }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    HAL_Delay(5);
}
