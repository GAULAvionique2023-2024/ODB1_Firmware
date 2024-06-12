/*
 * BMP280.c
 *
 *  Created on: May 18, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/BMP280.h"

#include <math.h>


uint8_t BMP280_ReadRegister(uint8_t reg);
uint8_t BMP280_WriteRegister(uint8_t reg, uint8_t value);
void BMP280_ReadCalibrationData(BMP280 *devBMP);
uint8_t BMP280_BMP280_MeasureReference(BMP280 devBMP, float temp_ref, float press_ref);

uint8_t BMP280_Init(BMP280 *devBMP, unsigned short spi_port) {

	SPI_Init(spi_port);

    BMP280_WriteRegister(BMP280_REG_RESET, BMP280_RESET_WORD); // Reset
    // Check ID
    if (BMP280_ReadRegister(BMP280_REG_ID) != BMP280_DEVICE_ID) {
        return 0; // Error
    }
    // Lire calibration
    BMP280_ReadCalibrationData(devBMP);
    // Configuration
    BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, BMP280_SETTING_CTRL_MEAS_NORMAL);
    BMP280_WriteRegister(BMP280_REG_CONFIG, BMP280_SETTING_CONFIG);
    // Ajuster reference
    BMP280_MeasureReference(devBMP, T0, 101325.0);

    return 1;
}

uint8_t BMP280_ReadTemperature(BMP280 *devBMP) {

	// Verifie si la lecture est possible
	while((BMP280_ReadRegister(BMP280_REG_STATUS) & 0x04) != 0 || (BMP280_ReadRegister(BMP280_REG_STATUS) & 0x01) != 0);

    int32_t adc_T = (BMP280_ReadRegister(BMP280_REG_TEMP_MSB) << 12) |
                    (BMP280_ReadRegister(BMP280_REG_TEMP_LSB) << 4) |
					((BMP280_ReadRegister(BMP280_REG_TEMP_XLSB) >> 4) & 0x0F);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)devBMP->calib_data.dig_T1 << 1))) * ((int32_t)devBMP->calib_data.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)devBMP->calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)devBMP->calib_data.dig_T1))) >> 12) * ((int32_t)devBMP->calib_data.dig_T3)) >> 14;
    devBMP->t_fine = var1 + var2;

    int32_t T = (devBMP->t_fine * 5 + 128) >> 8;
    devBMP->temp_C = (float)(T / 100.0); // + devBMP->temperature_ref;
    return 1; // OK
}

uint8_t BMP280_ReadPressure(BMP280 *devBMP) {

	// Verifie si la lecture est possible
	while((BMP280_ReadRegister(BMP280_REG_STATUS) & 0x04) != 0 || (BMP280_ReadRegister(BMP280_REG_STATUS) & 0x01) != 0);

    int32_t adc_P = (BMP280_ReadRegister(BMP280_REG_PRESS_MSB) << 12) |
                    (BMP280_ReadRegister(BMP280_REG_PRESS_LSB) << 4) |
                    ((BMP280_ReadRegister(BMP280_REG_PRESS_XLSB) >> 4) & 0x0F);

    int64_t var1 = ((int64_t)devBMP->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)devBMP->calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)devBMP->calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)devBMP->calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)devBMP->calib_data.dig_P3) >> 8) + ((var1 * (int64_t)devBMP->calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)devBMP->calib_data.dig_P1) >> 33;
    if (var1 == 0) {
        return 0; // Error (division par 0)
    }
    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)devBMP->calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)devBMP->calib_data.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)devBMP->calib_data.dig_P7) << 4);
    p = (uint32_t)p;

    devBMP->pressure_Pa = (float)p / (float)(1 << 8); // + devBMP->pressure_ref;
    return 1; // OK
}

void BMP280_ReadCalibrationData(BMP280 *devBMP) {

	// Verifie si la lecture est possible
	while((BMP280_ReadRegister(BMP280_REG_STATUS) & 0x04) != 0);

    uint8_t calib[26]; // 0x88 a 0xA1
    for (int i = 0; i < 26; i++) {
        calib[i] = BMP280_ReadRegister(BMP280_REG_CALIB_00 + i);
    }

    devBMP->calib_data.dig_T1 = (calib[1] << 8) | calib[0];
    devBMP->calib_data.dig_T2 = (calib[3] << 8) | calib[2];
    devBMP->calib_data.dig_T3 = (calib[5] << 8) | calib[4];
    devBMP->calib_data.dig_P1 = (calib[7] << 8) | calib[6];
    devBMP->calib_data.dig_P2 = (calib[9] << 8) | calib[8];
    devBMP->calib_data.dig_P3 = (calib[11] << 8) | calib[10];
    devBMP->calib_data.dig_P4 = (calib[13] << 8) | calib[12];
    devBMP->calib_data.dig_P5 = (calib[15] << 8) | calib[14];
    devBMP->calib_data.dig_P6 = (calib[17] << 8) | calib[16];
    devBMP->calib_data.dig_P7 = (calib[19] << 8) | calib[18];
    devBMP->calib_data.dig_P8 = (calib[21] << 8) | calib[20];
    devBMP->calib_data.dig_P9 = (calib[23] << 8) | calib[22];
}

uint8_t BMP280_SwapMode(uint8_t mode) {

	if(BMP280_ReadRegister(BMP280_REG_CTRL_MEAS) != mode) {
		BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, mode); // BMP280_SETTING_CTRL_MEAS_NORMAL (0x57) ou BMP280_SETTING_CTRL_MEAS_LOW (0x54)
		printf("BMP mode set to: %i/n", BMP280_SETTING_CTRL_MEAS_NORMAL);
		return 1; // OK
	} else {
		printf("BMP mode set error...");
		return 0; // Error (no change)
	}
}

uint8_t BMP280_MeasureReference(BMP280 *devBMP, float temp_ref, float press_ref) {

    devBMP->temperature_ref = BMP280_ReadTemperature(devBMP) - temp_ref;
    devBMP->pressure_ref = BMP280_ReadPressure(devBMP) - press_ref;
    return 1; // OK
}

float BMP280_PressureToAltitude(float pressure) {

	float altitude = (T0 / alpha) * (1 - pow((pressure / P0), (1 / beta)));
    return altitude;
}

uint8_t BMP280_ReadRegister(uint8_t reg) {

    uint8_t received_data;
    Write_GPIO(PA, 8, LOW); // Disable CS
    SPI2_TX(&reg, 1);
    SPI2_RX(&received_data, 1);
    Write_GPIO(PA, 8, HIGH); // Enable CS
    return received_data;
}

uint8_t BMP280_WriteRegister(uint8_t reg, uint8_t value) {

    uint8_t data[2] = {reg, value};
    Write_GPIO(PA, 8, LOW); // Disable CS
    SPI2_TX(data, 2);
    Write_GPIO(PA, 8, HIGH); // Enable CS
    return 1; // OK
}
