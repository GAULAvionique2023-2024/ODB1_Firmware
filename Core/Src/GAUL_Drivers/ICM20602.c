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

uint8_t ICM20602_Init(ICM20602 *dev) {
    dev->gyroXRaw = 0.0f;
    dev->gyroYRaw = 0.0f;
    dev->gyroZRaw = 0.0f;
    dev->accXRaw = 0.0f;
    dev->accYRaw = 0.0f;
    dev->accZRaw = 0.0f;
    dev->accResult = 0.0f;
    dev->temperatureC = 0.0f;

    Init_GPIO(dev->cs_port, dev->cs_pin, OUT50, O_GP_PP); // CS
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    Init_GPIO(dev->int_port, dev->int_pin, IN, I_PP); // Init GPIO for the interrupt

    uint8_t rxData;

    // Réinitialiser ICM20602
    ICM20602_Write(dev, ICM20602_REG_PWR_MGMT_1, 0x80);
    ICM20602_Read(dev, ICM20602_REG_WHO_AM_I, &rxData, 1);
    if (rxData != 0x12)
        return 0;

    // Verrouiller la communication SPI
    ICM20602_Write(dev, ICM20602_REG_I2C_IF, 0x40);
    ICM20602_Read(dev, ICM20602_REG_I2C_IF, &rxData, 1);
    if (rxData != 0x40)
        return 0;

    // Activer le capteur de température
    ICM20602_Write(dev, ICM20602_REG_PWR_MGMT_1, 0x01);
    ICM20602_Read(dev, ICM20602_REG_PWR_MGMT_1, &rxData, 1);
    if (rxData != 0x01)
        return 0;

    // Définir la fréquence d'échantillonnage à 1000Hz et appliquer un filtre logiciel
    ICM20602_Write(dev, ICM20602_REG_SMPLRT_DIV, 0x00);
    ICM20602_Read(dev, ICM20602_REG_SMPLRT_DIV, &rxData, 1);
    if (rxData != 0x00)
        return 0;

    // Gyro LPF fc 20Hz(bit2:0-100) à un taux d'échantillonnage de 1kHz
    ICM20602_Write(dev, ICM20602_REG_CONFIG, 0x05);
    ICM20602_Read(dev, ICM20602_REG_CONFIG, &rxData, 1);
    if (rxData != 0x05)
        return 0;

    // Gyro 2000DPS
    ICM20602_Write(dev, ICM20602_REG_GYRO_CONFIG, 0x18);
    ICM20602_Read(dev, ICM20602_REG_CONFIG, &rxData, 1);
    if (rxData != 0x18)
        return 0;

    // Sensibilité de l'accéléromètre 16g
    ICM20602_Write(dev, ICM20602_REG_ACCEL_CONFIG, 0x18);
    ICM20602_Read(dev, ICM20602_REG_ACCEL_CONFIG, &rxData, 1);
    if (rxData != 0x18)
        return 0;

    // ACCEL_CONFIG2 0x1D
    ICM20602_Write(dev, ICM20602_REG_ACCEL_CONFIG2, 0x03); // Acc FCHOICE 1kHz(bit3-0), DLPF fc 44.8Hz(bit2:0-011)
    ICM20602_Read(dev, ICM20602_REG_ACCEL_CONFIG2, &rxData, 1);
    if (rxData != 0x03)
        return 0;

    // Activer les interruptions
    ICM20602_Write(dev, ICM20602_REG_INT_ENABLE, 0x01);
    ICM20602_Read(dev, ICM20602_REG_INT_ENABLE, &rxData, 1);
    if (rxData != 0x01)
        return 0;

    ICM20602_Remove_DC_Offset(dev, 2);

    return 1;
}

void ICM20602_Update_All(ICM20602 *dev) {
    uint8_t rxData[14];

    // Lire les données brutes depuis le capteur
    ICM20602_Read(dev, ICM20602_REG_ACCEL_XOUT_H, rxData, 14);

    // Accéléromètre
    dev->accXRaw = (rxData[0] << 8) | rxData[1];
    dev->accYRaw = (rxData[2] << 8) | rxData[3];
    dev->accZRaw = (rxData[4] << 8) | rxData[5];

    // Température
    dev->temperatureC = ((rxData[6] << 8) | rxData[7]) / 326.8f + 25;

    // Gyroscope
    dev->gyroXRaw = (rxData[8] << 8) | rxData[9];
    dev->gyroYRaw = (rxData[10] << 8) | rxData[11];
    dev->gyroZRaw = (rxData[12] << 8) | rxData[13];

    // Conversion des valeurs brutes en unités
    dev->gyroX = dev->gyroXRaw * 2000.f / 32768.f;
    dev->gyroY = dev->gyroYRaw * 2000.f / 32768.f;
    dev->gyroZ = dev->gyroZRaw * 2000.f / 32768.f;

    dev->accX = dev->accXRaw * 16.f / 32768.f;
    dev->accY = dev->accYRaw * 16.f / 32768.f;
    dev->accZ = dev->accZRaw * 16.f / 32768.f;
    dev->accResult = sqrt(pow(dev->accX, 2) + pow(dev->accY, 2) + pow(dev->accZ, 2));

    // Calcul des angles Roll et Pitch
    dev->angleRoll = atan(dev->accY / sqrt(dev->accX * dev->accX + dev->accZ * dev->accZ)) * (180 / M_PI);
    dev->anglePitch = -atan(dev->accX / sqrt(dev->accY * dev->accY + dev->accZ * dev->accZ)) * (180 / M_PI);

    // Appeler une fonction externe pour obtenir Roll et Pitch
    getRollPitch(dev);
}

void ICM20602_Remove_DC_Offset(ICM20602 *dev, uint8_t mean) {
    int16_t offset[3] = { 0, 0, 0 };
    uint8_t rxData[6];

    for (int8_t i = 0; i < mean; i++) {
        // Lire les données brutes du gyroscope
        ICM20602_Read(dev, ICM20602_REG_GYRO_XOUT_H, rxData, 6);

        offset[0] += (rxData[0] << 8) | rxData[1];
        offset[1] += (rxData[2] << 8) | rxData[3];
        offset[2] += (rxData[4] << 8) | rxData[5];
    }

    offset[0] /= mean;
    offset[1] /= mean;
    offset[2] /= mean;

    // Écrire les offsets dans les registres du gyroscope
    ICM20602_Write(dev, ICM20602_REG_XG_OFFS_USRH, (offset[0] * -2) >> 8);
    ICM20602_Write(dev, ICM20602_REG_XG_OFFS_USRL, offset[0] * -2);

    ICM20602_Write(dev, ICM20602_REG_YG_OFFS_USRH, (offset[1] * -2) >> 8);
    ICM20602_Write(dev, ICM20602_REG_YG_OFFS_USRL, offset[1] * -2);

    ICM20602_Write(dev, ICM20602_REG_ZG_OFFS_USRH, (offset[2] * -2) >> 8);
    ICM20602_Write(dev, ICM20602_REG_ZG_OFFS_USRL, offset[2] * -2);
}

int8_t ICM20602_Data_Ready(ICM20602 *dev) {
    return Read_GPIO(dev->int_port, dev->int_pin);
}

void ICM20602_Read(ICM20602 *dev, uint8_t address, uint8_t rxData[], uint8_t size) {
    address |= 0x80;  // read operation

    Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0) { /* Handle timeout error */
        Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
        return;
    }
    if (SPI_RX(dev->SPIx, rxData, size) != 0) {/* Handle timeout error */
        Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
        return;
    }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
}

void ICM20602_Write(ICM20602 *dev, uint8_t address, uint8_t value) {
    address &= 0x7F;  // Write operation

    Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0) { /* Handle timeout error */
    }
    if (SPI_TX(dev->SPIx, &value, 1) != 0) { /* Handle timeout error */
    }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    HAL_Delay(20);
}
