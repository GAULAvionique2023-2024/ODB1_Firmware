/*
 * BMP280_tests.c
 *
 * BMP280_Init() doit être exécuté avant les fonctions du module.
 *
 *  Created on: Jul 8, 2024
 *      Author: mathouqc
 */

#include "GAUL_Drivers/Tests/BMP280_tests.h"

#include "GAUL_Drivers/BMP280.h"
#include <stdio.h>

extern BMP280 bmp_data;

void BMP280_TESTS_LogData() {
    BMP280_Read_Temperature_Pressure(&bmp_data);

    printf("Measures:   %8.4f kPa at %5.2f C\n", bmp_data.pressure_Pa / 1000, bmp_data.temp_C);
    printf("Alt rel:    %8.2f m\n", bmp_data.altitude_m);
    printf("Alt filter: %8.2f m\n", bmp_data.altitude_filtered_m);
    printf("Alt MSL:    %8.2f m\n", bmp_data.altitude_MSL);
    printf("\n");
}

void BMP280_TESTS_LogCalib() {
    BMP280_Read_Calib_Data(&bmp_data);

    printf("dig_T1: %du\n", bmp_data.calib_data.dig_T1);
    printf("dig_T2: %d\n", bmp_data.calib_data.dig_T2);
    printf("dig_T3: %d\n", bmp_data.calib_data.dig_T3);
    printf("dig_P1: %du\n", bmp_data.calib_data.dig_P1);
    printf("dig_P2: %d\n", bmp_data.calib_data.dig_P2);
    printf("dig_P3: %d\n", bmp_data.calib_data.dig_P3);
    printf("dig_P4: %d\n", bmp_data.calib_data.dig_P4);
    printf("dig_P5: %d\n", bmp_data.calib_data.dig_P5);
    printf("dig_P6: %d\n", bmp_data.calib_data.dig_P6);
    printf("dig_P7: %d\n", bmp_data.calib_data.dig_P7);
    printf("dig_P8: %d\n", bmp_data.calib_data.dig_P8);
    printf("dig_P9: %d\n", bmp_data.calib_data.dig_P9);
    printf("\n");
}

void BMP280_TESTS_All() {
    BMP280_TESTS_LogCalib();
    HAL_Delay(2000);
    while (1) {
        BMP280_TESTS_LogData();
        HAL_Delay(500);
    }
}
