/*
 * L76LM33_tests.c
 *
 *  Created on: Jul 30, 2024
 *      Author: mathouqc
 */

#include "GAUL_Drivers/Tests/L76LM33_tests.h"

#include <stdio.h>
#include <string.h>

extern L76LM33 L76_data;

void L76LM33_TESTS_ReadSentence() {
    if (L76LM33_Read_Sentence(&L76_data) == L76LM33_OK) {
        printf("%s\n", L76_data.NMEA_Buffer);
    }
}

void L76LM33_TESTS_Read() {
    if (L76LM33_Read(&L76_data) == 0) {
        printf("%s\n", L76_data.NMEA_Buffer);
        L76LM33_TESTS_LogStructure(&L76_data);
    }
}

void L76LM33_TESTS_LogStructure(L76LM33 *L76_data) {
    printf("Status:  %s\n", L76_data->state == 1 ? "OK" : "Error");
    printf("Fix:  %s\n", L76_data->gps_data.fix == 1 ? "Yes" : "No");
    printf("Lat:  %f\n", L76_data->gps_data.latitude);
    printf("Lon:  %f\n", L76_data->gps_data.longitude);
    printf("\n");
}
