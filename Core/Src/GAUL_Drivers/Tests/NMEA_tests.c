/*
 * NMEA_tests.c
 *
 *  Created on: Jul 25, 2024
 *      Author: mathouqc
 */

#include "GAUL_Drivers/Tests/NMEA_tests.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static GPS_Data gps_data;

void NMEA_TESTS_ValidateRMC_Log() {
    if (NMEA_ValidateRMC("$GNRMC,080608.000,A") == 0) {
        printf("Test 1 passed\n");
    } else {
        printf("Test 1 failed\n");
    }

    if (NMEA_ValidateRMC("$GPRMC,080608.000,A") == 0) {
        printf("Test 2 passed\n");
    } else {
        printf("Test 2 failed\n");
    }

    if (NMEA_ValidateRMC("$GPGGA,080608.000,A") == -1) {
        printf("Test 3 passed\n");
    } else {
        printf("Test 3 failed\n");
    }

    if (NMEA_ValidateRMC("$G") == -1) {
        printf("Test 4 passed\n");
    } else {
        printf("Test 4 failed\n");
    }
}

void NMEA_TESTS_ParseRMC_Log() {
    char sentence[120];

    // Test 1
    strncpy(sentence, "$GNRMC,080608.000,A,3029.461489,N,11430.072002,E,0.00,148.41,210423,,,D,V*09", 120);
    printf("%s\n", sentence);
    NMEA_ParseRMC(&gps_data, sentence);

    NMEA_TESTS_LogStructure(&gps_data);

    if (gps_data.time.hours == 8 && gps_data.time.minutes == 6 && fabs(gps_data.time.seconds - 8.0) < 0.001 && gps_data.fix == 1
            && fabs(gps_data.latitude - 30.491024) < 0.000002 // 2e-6 because of rounding error. 1e-6 deg ~ 11cm ground error
            && fabs(gps_data.longitude - 114.501200) < 0.000002) { // 2e-6 rounding error ??
        printf("Test 1 passed\n");
    } else {
        printf("Test 1 failed\n");
    }

    printf("\n");

    // Test 2
    strncpy(sentence, "$GNRMC,185609.020,V,,,,,,,,,,,V*09", 120);
    printf("%s\n", sentence);
    NMEA_ParseRMC(&gps_data, sentence);

    NMEA_TESTS_LogStructure(&gps_data);

    if (gps_data.time.hours == 18 && gps_data.time.minutes == 56 && fabs(gps_data.time.seconds - 9.02) < 0.001 && gps_data.fix == 0
            && fabs(gps_data.latitude - 0) < 0.000002 && fabs(gps_data.longitude - 0) < 0.000002) {
        printf("Test 2 passed\n");
    } else {
        printf("Test 2 failed\n");
    }

    printf("\n");

    // Test 3
    strncpy(sentence, "$GNRMC,124631,A,3159.99994,N,07100.0000,W,0.00,34.91,210423,,,D,V*09", 120);
    printf("%s\n", sentence);
    NMEA_ParseRMC(&gps_data, sentence);

    NMEA_TESTS_LogStructure(&gps_data);

    if (gps_data.time.hours == 12 && gps_data.time.minutes == 46 && fabs(gps_data.time.seconds - 31) < 0.001 && gps_data.fix == 1
            && fabs(gps_data.latitude - 31.999999) < 0.000002 && fabs(gps_data.longitude - -71) < 0.000002) {
        printf("Test 3 passed\n");
    } else {
        printf("Test 3 failed\n");
    }

    printf("\n");

    // Test 4
    strncpy(sentence, "$GNRMC,124631,A,3159.99994,N,07100.0000,W", 120);
    printf("%s\n", sentence);
    NMEA_ParseRMC(&gps_data, sentence);

    NMEA_TESTS_LogStructure(&gps_data);

    if (gps_data.time.hours == 12 && gps_data.time.minutes == 46 && fabs(gps_data.time.seconds - 31) < 0.001 && gps_data.fix == 1
            && fabs(gps_data.latitude - 31.999999) < 0.000002 && fabs(gps_data.longitude - -71) < 0.000002) {
        printf("Test 4 passed\n");
    } else {
        printf("Test 4 failed\n");
    }

    printf("\n");

    // Test 5
    strncpy(sentence, "$GNRMC,1246031,A,3159.99994,N,07100.0000,W", 120); // Error in time
    printf("%s\n", sentence);
    if (NMEA_ParseRMC(&gps_data, sentence) != 0) {
        printf("Test 5 passed\n");
    } else {
        printf("Test 5 failed\n");
    }

    printf("\n");

    // Test 6
    strncpy(sentence, "$GNRMC,124631,A,31590.9994,N,07100.0000,W", 120); // Error in lat (extra char)
    printf("%s\n", sentence);
    if (NMEA_ParseRMC(&gps_data, sentence) != 0) {
        printf("Test 6 passed\n");
    } else {
        printf("Test 6 failed\n");
    }

    printf("\n");

    // Test 7
    strncpy(sentence, "$GNRMC,124631,A,3159.9994,N,0710.0000,W", 120); // Error in lon (missing char)
    printf("%s\n", sentence);
    if (NMEA_ParseRMC(&gps_data, sentence) != 0) {
        printf("Test 7 passed\n");
    } else {
        printf("Test 7 failed\n");
    }

    printf("\n");

    // Test 8
    strncpy(sentence, "$GNRMC,124631,A,3159.9994,T,07100.0000,W", 120); // Error in indicator
    printf("%s\n", sentence);
    if (NMEA_ParseRMC(&gps_data, sentence) != 0) {
        printf("Test 8 passed\n");
    } else {
        printf("Test 8 failed\n");
    }
}

void NMEA_TESTS_LogStructure(GPS_Data *gps_data) {
    printf("Time raw: %ld\n", gps_data->time_raw);
    printf("Time: %02d:%02d:%06.3f\n", gps_data->time.hours, gps_data->time.minutes, gps_data->time.seconds);
    printf("Fix:  %s\n", gps_data->fix == 1 ? "Yes" : "No");
    printf("Lat:  %f\n", gps_data->latitude);
    printf("Lon:  %f\n", gps_data->longitude);
}
