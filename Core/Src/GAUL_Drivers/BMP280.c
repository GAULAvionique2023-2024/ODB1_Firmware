/*
 * BMP280.c
 *
 * Le BMP280 est un baromètre utilisé pour avoir l'altitude de la fusée.
 *
 *  Created on: May 18, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/BMP280.h"

#include <math.h> // pour pow()

uint8_t BMP280_Init(BMP280 *dev) {
    Init_GPIO(dev->cs_port, dev->cs_pin, OUT50, O_GP_PP); // CS
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    // Soft reset
    BMP280_Write(dev, BMP280_REG_RESET, BMP280_RESET_WORD);
    HAL_Delay(2); // startup time in data sheet (2ms)

    // Check ID
    uint8_t id;
    BMP280_Read(dev, BMP280_REG_ID, &id, sizeof(id));
    if (id != BMP280_DEVICE_ID) {
        return 1; // Error
    }

    // Lire les données de calibration
    BMP280_Read_Calib_Data(dev);

    // Configurer les paramètres de mesure
    BMP280_Write(dev, BMP280_REG_CTRL_MEAS, BMP280_SETTING_CTRL_MEAS_NORMAL);
    BMP280_Write(dev, BMP280_REG_CONFIG, BMP280_SETTING_CONFIG_NORMAL);

    // Lire la pression de référence au démarrage
    BMP280_Read_Temperature_Pressure(dev);
    dev->pressure_ref = dev->pressure_Pa;

    // Initialiser l'altitude filtrée avec l'altitude initiale
    dev->altitude_filtered_m = 0.0f;
    dev->alpha = BMP280_FILTER_FACTOR; // Ajustez ce facteur selon le niveau de lissage souhaité (entre 0 et 1)

    return 0;
}

void BMP280_Read_Temperature_Pressure(BMP280 *dev) {
    uint8_t data[6];

    // Lire les données de pression et de température
    BMP280_Read(dev, BMP280_REG_PRESS_MSB, data, 6);

    // Combiner les octets de données
    int32_t raw_pressure = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    int32_t raw_temperature = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);

    // Application de la compensation de température
    int32_t var1, var2, t_fine;

    int32_t dig_T1 = dev->calib_data.dig_T1;
    int32_t dig_T2 = dev->calib_data.dig_T2;
    int32_t dig_T3 = dev->calib_data.dig_T3;

    var1 = ((((raw_temperature >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    var2 = (((((raw_temperature >> 4) - ((int32_t)dig_T1)) * ((raw_temperature >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;

    dev->temp_C = (t_fine * 5 + 128) >> 8;
    dev->temp_C /= 100.0f; // Convertir en degrés Celsius

    // Application de la compensation de pression
    int64_t p_var1, p_var2, p;
    int64_t dig_P1 = dev->calib_data.dig_P1;
    int64_t dig_P2 = dev->calib_data.dig_P2;
    int64_t dig_P3 = dev->calib_data.dig_P3;
    int64_t dig_P4 = dev->calib_data.dig_P4;
    int64_t dig_P5 = dev->calib_data.dig_P5;
    int64_t dig_P6 = dev->calib_data.dig_P6;
    int64_t dig_P7 = dev->calib_data.dig_P7;
    int64_t dig_P8 = dev->calib_data.dig_P8;
    int64_t dig_P9 = dev->calib_data.dig_P9;

    p_var1 = ((int64_t)t_fine) - 128000;
    p_var2 = p_var1 * p_var1 * (int64_t)dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dig_P3) >> 8) + ((p_var1 * (int64_t)dig_P2) << 12);
    p_var1 = ((((int64_t)1) << 47) + p_var1) * ((int64_t)dig_P1) >> 33;

    if (p_var1 == 0) {
        dev->pressure_Pa = 0; // Prévenir la division par zéro
    } else {
        p = 1048576 - raw_pressure;
        p = (((p << 31) - p_var2) * 3125) / p_var1;
        p_var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        p_var2 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dig_P7) << 4);
        dev->pressure_Pa = (float)p / 256.0f; // Convertir en Pascals
    }

    // Calcul de l'altitude relative
    float pressure_ratio = dev->pressure_Pa / dev->pressure_ref;
    dev->altitude_m = 44330.0f * (1.0f - pow(pressure_ratio, 0.1903f));

    // Appliquer le filtre EMA
    dev->altitude_filtered_m = dev->alpha * dev->altitude_m + (1 - dev->alpha) * dev->altitude_filtered_m;

    // Convertir la pression en altitude
    dev->altitude_MSL = BMP280_PressureToAltitude(dev->pressure_Pa / 100.0f, BMP280_HPA_SEA_LEVEL); // Convertir la pression en hPa et utiliser la pression au niveau de la mer par défaut
}

float BMP280_PressureToAltitude(float pressure, float sea_level_pressure) {
    // Constants
    const float T0 = 288.15;     // Standard temperature at sea level in Kelvin
    const float L = 0.0065;      // Temperature lapse rate in K/m
    const float R = 287.05;      // Specific gas constant for dry air in J/(kg·K)
    const float g = 9.80665;     // Gravitational acceleration in m/s^2

    // Calculate the altitude
    float altitude = (T0 / L) * (1 - pow((pressure / sea_level_pressure), (R * L) / g));

    return altitude;
}

void BMP280_Read_Calib_Data(BMP280 *dev) {
    uint8_t calib[24];
    BMP280_Read(dev, BMP280_REG_CALIB_00, calib, 24);

    dev->calib_data.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    dev->calib_data.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    dev->calib_data.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    dev->calib_data.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    dev->calib_data.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    dev->calib_data.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    dev->calib_data.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    dev->calib_data.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    dev->calib_data.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    dev->calib_data.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    dev->calib_data.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    dev->calib_data.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

void BMP280_Write(BMP280 *dev, uint8_t address, uint8_t value) {
    address &= 0x7F;  // Write operation

    Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0) { /* Handle timeout error */
    }
    if (SPI_TX(dev->SPIx, &value, 1) != 0) { /* Handle timeout error */
    }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);

    HAL_Delay(20);
}

void BMP280_Read(BMP280 *dev, uint8_t address, uint8_t *rxData[], uint8_t size) {

    address |= 0x80;  // read operation

    Write_GPIO(dev->cs_port, dev->cs_pin, LOW);
    if (SPI_TX(dev->SPIx, &address, 1) != 0) {
        Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
        return;
    }
    if (SPI_RX(dev->SPIx, rxData, size) != 0) {
        Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
        return;
    }
    Write_GPIO(dev->cs_port, dev->cs_pin, HIGH);
}

uint8_t BMP280_SwapMode(uint8_t mode) {

    if (BMP280_ReadRegister(BMP280_REG_CTRL_MEAS) != mode) {
        BMP280_WriteRegister(BMP280_REG_CTRL_MEAS, mode); // BMP280_SETTING_CTRL_MEAS_NORMAL (0x57) or BMP280_SETTING_CTRL_MEAS_LOW (0x54)
        return 1; // OK
    } else {
        return 0; // Error (no change)
    }
}
