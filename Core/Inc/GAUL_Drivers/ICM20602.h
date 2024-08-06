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

#define ICM20602_REG_XG_OFFS_TC_H			0x04
#define ICM20602_REG_XG_OFFS_TC_L			0x05
#define ICM20602_REG_YG_OFFS_TC_H			0x07
#define ICM20602_REG_YG_OFFS_TC_L			0x08
#define ICM20602_REG_ZG_OFFS_TC_H			0x0A
#define ICM20602_REG_ZG_OFFS_TC_L			0x0B
#define ICM20602_REG_SELF_TEST_X_ACCEL		0x0D
#define ICM20602_REG_SELF_TEST_Y_ACCEL		0x0E
#define ICM20602_REG_SELF_TEST_Z_ACCEL		0x0F
#define ICM20602_REG_XG_OFFS_USRH			0x13
#define ICM20602_REG_XG_OFFS_USRL			0x14
#define ICM20602_REG_YG_OFFS_USRH			0x15
#define ICM20602_REG_YG_OFFS_USRL			0x16
#define ICM20602_REG_ZG_OFFS_USRH			0x17
#define ICM20602_REG_ZG_OFFS_USRL			0x18
#define ICM20602_REG_SMPLRT_DIV				0x19
#define ICM20602_REG_CONFIG					0x1A
#define ICM20602_REG_GYRO_CONFIG			0x1B
#define ICM20602_REG_ACCEL_CONFIG			0x1C
#define ICM20602_REG_ACCEL_CONFIG2			0x1D
#define ICM20602_REG_LP_MODE_CFG			0x1E
#define ICM20602_REG_ACCEL_WOM_X_THR		0x20
#define ICM20602_REG_ACCEL_WOM_Y_THR		0x21
#define ICM20602_REG_ACCEL_WOM_Z_THR		0x22
#define ICM20602_REG_FIFO_EN				0x23
#define ICM20602_REG_FSYNC_INT				0x36
#define ICM20602_REG_INT_PIN_CFG			0x37
#define ICM20602_REG_INT_ENABLE				0x38
#define ICM20602_REG_FIFO_WM_INT_STATUS		0x39
#define ICM20602_REG_INT_STATUS				0x3A
#define ICM20602_REG_ACCEL_XOUT_H			0x3B
#define ICM20602_REG_ACCEL_XOUT_L			0x3C
#define ICM20602_REG_ACCEL_YOUT_H			0x3D
#define ICM20602_REG_ACCEL_YOUT_L			0x3E
#define ICM20602_REG_ACCEL_ZOUT_H			0x3F
#define ICM20602_REG_ACCEL_ZOUT_L			0x40
#define ICM20602_REG_TEMP_OUT_H				0x41
#define ICM20602_REG_TEMP_OUT_L				0x42
#define ICM20602_REG_GYRO_XOUT_H			0x43
#define ICM20602_REG_GYRO_XOUT_L			0x44
#define ICM20602_REG_GYRO_YOUT_H			0x45
#define ICM20602_REG_GYRO_YOUT_L			0x46
#define ICM20602_REG_GYRO_ZOUT_H			0x47
#define ICM20602_REG_GYRO_ZOUT_L			0x48
#define ICM20602_REG_SELF_TEST_X_GYRO		0x50
#define ICM20602_REG_SELF_TEST_Y_GYRO		0x51
#define ICM20602_REG_SELF_TEST_Z_GYRO		0x52
#define ICM20602_REG_FIFO_WM_TH1			0x60
#define ICM20602_REG_FIFO_WM_TH2			0x61
#define ICM20602_REG_SIGNAL_PATH_RESET		0x68
#define ICM20602_REG_ACCEL_INTEL_CTRL		0x69
#define ICM20602_REG_USER_CTRL				0x6A
#define ICM20602_REG_PWR_MGMT_1				0x6B
#define ICM20602_REG_PWR_MGMT_2				0x6C
#define ICM20602_REG_I2C_IF					0x70
#define ICM20602_REG_FIFO_COUNTH			0x72
#define ICM20602_REG_FIFO_COUNTL			0x73
#define ICM20602_REG_FIFO_R_W				0x74
#define ICM20602_REG_WHO_AM_I				0x75
#define ICM20602_REG_XA_OFFSET_H			0x77
#define ICM20602_REG_XA_OFFSET_L			0x78
#define ICM20602_REG_YA_OFFSET_H			0x7A
#define ICM20602_REG_YA_OFFSET_L			0x7B
#define ICM20602_REG_ZA_OFFSET_H			0x7D
#define ICM20602_REG_ZA_OFFSET_L			0x7E


#define ICM20602_VAL_WHO_AM_I 				0x12

// Sensor struct
typedef struct{

	SPI_TypeDef *SPIx;
	uint8_t 	cs_pin;
	GPIO_TypeDef *cs_port;

	uint8_t 	int_pin;
	GPIO_TypeDef *int_port;

	// Raw data
	int16_t 	gyroXRaw;
	int16_t 	gyroYRaw;
	int16_t 	gyroZRaw;

	int16_t 	accXRaw;
	int16_t 	accYRaw;
	int16_t 	accZRaw;

	// Real data
	float 	gyroX;
	float 	gyroY;
	float 	gyroZ;

	float 	accX;
	float 	accY;
	float 	accZ;
	float 	accResult;

	float 	temperatureC;

	float	angleRoll;
	float	anglePitch;
	float	kalmanAngleRoll; // Gauche/Droite
	float	kalmanAnglePitch; // Haut/Bas

}ICM20602;

uint8_t ICM20602_Init(ICM20602 *dev);

void 	ICM20602_Update_All(ICM20602 *dev);
void 	ICM20602_Remove_DC_Offset(ICM20602 *dev, uint8_t mean);
int8_t ICM20602_Data_Ready(ICM20602 *dev);

//Low level fonctions
void ICM20602_Read(ICM20602 *dev, uint8_t address, uint8_t rxData[], uint8_t size);
void ICM20602_Write(ICM20602 *dev, uint8_t address, uint8_t value);

#endif /* INC_ICM20602_H_*/
