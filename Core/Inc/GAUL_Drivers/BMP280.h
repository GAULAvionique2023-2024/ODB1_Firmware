/*
 * BMP280.h
 *
 *  Created on: May 18, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "stm32f1xx_hal.h"

#include <stdint.h>
#include <stdio.h>

#ifndef INC_GAUL_DRIVERS_BMP280_H_
#define INC_GAUL_DRIVERS_BMP280_H_

#define BMP280_DEVICE_ID        0x58
#define BMP280_RESET_WORD       0xB6

#define BMP280_REG_TEMP_XLSB    0xFC
#define BMP280_REG_TEMP_LSB     0xFB
#define BMP280_REG_TEMP_MSB     0xFA
#define BMP280_REG_PRESS_XLSB   0xF9
#define BMP280_REG_PRESS_LSB    0xF8
#define BMP280_REG_PRESS_MSB    0xF7
#define BMP280_REG_CONFIG       0xF5
#define BMP280_REG_CTRL_MEAS    0xF4
#define BMP280_REG_STATUS       0xF3
#define BMP280_REG_RESET        0xE0
#define BMP280_REG_ID           0xD0
#define BMP280_REG_CALIB_00 	0x88

//Setting ctrl_meas register (data acquisition options -> indoor navigation osrs_p x16, osrs_t x2, irrcoeff 16)
//Temperature (osrs_t)    17bit resolution    010 (± 0.0025)
//Pressure (osrs_p)       20bit resolution    101 (± 0.0003)
//Power mode              normal              11  (sleep 00 ; force 01/10 ; normal 11)
//010;101;11 = 0x57 (normal)
#define BMP280_SETTING_CTRL_MEAS_NORMAL 0x57
#define BMP280_SETTING_CTRL_MEAS_LOW 0x54

//Setting config register (date, filter, interfaces options)
//Stanby time    0.5ms   000
//IIR filter     4x      010
//Bit 1          N/A     0
//Spi3w          4wire   0
//00001000 = 0x08
#define BMP280_SETTING_CONFIG 0x08

#define T0 288.15  		// Temperature mer (kelvin)
#define alpha 0.0065  	// Taux de temperature (km/h)
#define P0 101325.0  	// Pression atmospherique mer
#define beta 5.256  	// Indice temperature


typedef struct {
    uint16_t 	dig_T1;
    int16_t 	dig_T2;
    int16_t 	dig_T3;
    uint16_t 	dig_P1;
    int16_t 	dig_P2;
    int16_t 	dig_P3;
    int16_t 	dig_P4;
    int16_t 	dig_P5;
    int16_t 	dig_P6;
    int16_t 	dig_P7;
    int16_t 	dig_P8;
    int16_t 	dig_P9;
} BMP280_CalibData;

typedef struct {
    float 				pressure_Pa;
    float 				temp_C;
    BMP280_CalibData 	calib_data;
    int32_t 			t_fine;
    float 				temperature_ref;
    float 				pressure_ref;
} BMP280;

uint8_t BMP280_Init(BMP280 *devBMP, unsigned short spi_port, float temp_ref, float press_ref);

uint8_t BMP280_ReadTemperature(BMP280 *devBMP);
uint8_t BMP280_ReadPressure(BMP280 *devBMP);

uint8_t BMP280_SwapMode(uint8_t mode);
uint8_t BMP280_MeasureReference(BMP280 *devBMP, float temp_ref, float press_ref);
float BMP280_PressureToAltitude(float pressure);

uint8_t BMP280_ReadRegister(uint8_t reg);
uint8_t BMP280_WriteRegister(uint8_t reg, uint8_t value);

#endif /* INC_GAUL_DRIVERS_BMP280_H_ */
