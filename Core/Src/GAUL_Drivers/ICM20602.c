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

static float yaw = 0.0f;

static void anglesToRotationMatrix(float pitch, float roll, float yaw, float R[3][3]) {
    float sp = sinf(pitch);
    float cp = cosf(pitch);
    float sr = sinf(roll);
    float cr = cosf(roll);
    float sy = sinf(yaw);
    float cy = cosf(yaw);

    R[0][0] = cp * cy;
    R[0][1] = cp * sy;
    R[0][2] = -sp;

    R[1][0] = sr * sp * cy - cr * sy;
    R[1][1] = sr * sp * sy + cr * cy;
    R[1][2] = sr * cp;

    R[2][0] = cr * sp * cy + sr * sy;
    R[2][1] = cr * sp * sy - sr * cy;
    R[2][2] = cr * cp;
}

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

void ICM20602_Update_All(ICM20602 *dev) {
    static const float dt = 0.01f; // 100 Hz
    static const float alpha_vel = 0.98f; // filtre

    if (!ICM20602_Data_Ready(dev)) return;

    uint8_t rxData[14];
    int16_t gyroRawX, gyroRawY, gyroRawZ;
    int16_t accRawX, accRawY, accRawZ;

    ICM20602_Read(dev, ICM20602_REG_ACCEL_XOUT_H, rxData, 14);

    accRawX = (int16_t)((rxData[0] << 8) | rxData[1]);
    accRawY = (int16_t)((rxData[2] << 8) | rxData[3]);
    accRawZ = (int16_t)((rxData[4] << 8) | rxData[5]);

    dev->temperatureC = ((rxData[6] << 8) | rxData[7]) / 326.8f + 25;

    gyroRawX = (int16_t)((rxData[8] << 8) | rxData[9]);
    gyroRawY = (int16_t)((rxData[10] << 8) | rxData[11]);
    gyroRawZ = (int16_t)((rxData[12] << 8) | rxData[13]);

    // Gyro en deg/s
    dev->gyroX = gyroRawX * 2000.f / 32768.f;
    dev->gyroY = gyroRawY * 2000.f / 32768.f;
    dev->gyroZ = gyroRawZ * 2000.f / 32768.f;

    // Accélération en g
    dev->accX = accRawX * 16.f / 32768.f;
    dev->accY = accRawY * 16.f / 32768.f;
    dev->accZ = accRawZ * 16.f / 32768.f;

    dev->accResult = sqrtf(dev->accX * dev->accX +
                          dev->accY * dev->accY +
                          dev->accZ * dev->accZ);

    // Angles pitch et roll à partir de l'acc
    dev->angle_pitch_acc = -(atan2f(dev->accX, sqrtf(dev->accY * dev->accY + dev->accZ * dev->accZ)) * 180.0f) / M_PI;
    dev->angle_roll_acc  =  (atan2f(dev->accY, dev->accZ) * 180.0f) / M_PI;

    // Mise à jour filtres Kalman pitch et roll
    dev->kalmanPitch = KalmanFilter_Update(&kalmanPitch, dev->angle_pitch_acc, dev->gyroY);
    dev->kalmanRoll  = KalmanFilter_Update(&kalmanRoll, dev->angle_roll_acc, dev->gyroX);

    // Estimation yaw par intégration gyroZ (deg/s)
    yaw += dev->gyroZ * dt;
    if (yaw > 180.0f) yaw -= 360.0f;
    else if (yaw < -180.0f) yaw += 360.0f;

    dev->kalmanYaw = yaw;

    // Conversion angles en radians
    float pitch = dev->kalmanPitch * M_PI / 180.0f;
    float roll = dev->kalmanRoll * M_PI / 180.0f;
    float yaw_rad = yaw * M_PI / 180.0f;

    // Calcul matrice rotation capteur->monde
    float R[3][3];
    anglesToRotationMatrix(pitch, roll, yaw_rad, R);

    // Accélération en m/s²
    float acc_mps2[3] = {
        dev->accX * ICM20602_G_TO_V,
        dev->accY * ICM20602_G_TO_V,
        dev->accZ * ICM20602_G_TO_V
    };

    // Transformation vers repère monde
    float acc_world[3] = {
        R[0][0]*acc_mps2[0] + R[0][1]*acc_mps2[1] + R[0][2]*acc_mps2[2],
        R[1][0]*acc_mps2[0] + R[1][1]*acc_mps2[1] + R[1][2]*acc_mps2[2],
        R[2][0]*acc_mps2[0] + R[2][1]*acc_mps2[1] + R[2][2]*acc_mps2[2]
    };

    // Soustraction gravité (Z vers le haut)
    acc_world[2] -= ICM20602_G_TO_V;

    dev->velX = alpha_vel * (dev->velX + acc_world[0] * dt);
    dev->velY = alpha_vel * (dev->velY + acc_world[1] * dt);
    dev->velZ = alpha_vel * (dev->velZ + acc_world[2] * dt);
}

void ICM20602_Calibrate(ICM20602 *dev, int8_t p_Sense){
    uint8_t rxData[6];
    int16_t gyroRawX, gyroRawY, gyroRawZ;
    int16_t xOffset = 0;
    int16_t yOffset = 0;
    int16_t zOffset = 0;

    do {
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
