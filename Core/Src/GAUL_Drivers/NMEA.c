/*
 * NMEA.c
 *
 * Module to parse the time and the latitude/longitude from a RMC NMEA sentence.
 *
 *  Created on: May 12, 2024
 *      Author: gagnon
 *
 *  Edited on: Jul 4, 2024
 *      Autor: mathouqc
 */

#include "GAUL_Drivers/NMEA.h"

#include <string.h>
#include <stdlib.h>

/**
 * Validate the NMEA sentence ID is RMC ($xxRMC).
 * $GNRMC,080608.000,A,3029.461489,N,11430.072002,E,0.00,148.41,210423,,,D,V*09
 *
 * Takes 0.002ms to complete
 *
 * @param nmea_sentence: pointer to sentence array.
 *
 * @retval NMEA_OK RMC sentence
 * @retval NMEA_ERROR Not a RMC sentence
 */
int8_t NMEA_ValidateRMC(const char *nmea_sentence) {
    return strncmp(nmea_sentence+3, "RMC", 3) == 0 ? NMEA_OK : NMEA_ERROR;
}

/**
 * Parse NMEA RMC sentence ($xxRMC).
 * $GNRMC,080608.000,A,3029.461489,N,11430.072002,E,0.00,148.41,210423,,,D,V*09
 *
 * Latitude and longitude are 0.0 if there's no GPS fix.
 * See GPS_Data struct for more details.
 *
 * Takes 0.13ms to complete
 *
 * @param nmea_sentence: pointer to sentence array.
 * @param gps_data: pointer to structure to fill with parsed data.
 *
 * @retval 0 OK
 * @retval NMEA_ERROR ERROR
 */
int8_t NMEA_ParseRMC(GPS_Data *gps_data, const char *nmea_sentence) {

    if (!nmea_sentence || !gps_data) {
        return NMEA_ERROR; // Error, NULL sentence or structure
    }

    // Copy because strtok messes the string by replacing delimiter with \0
    // and causes HardFault when called on literal string nmea_sentence
    char *copy = strndup(nmea_sentence, NMEA_MAX_RMC_LENGTH);
    if (!copy) {
        return NMEA_ERROR; // Error while duplicating string
    }

    uint8_t tok_idx = 0; // Current field index
    char *token;

    // Read first field
    token = strtok(copy, ",");

    // Only read lat/lon and avoid infinite loop
    while (token != NULL && tok_idx < NMEA_MAX_TOKEN_TO_READ) {
        if (tok_idx == 1) { // TIME
            if (strnlen(token, 6) < 6) { // hhmmss.sss = 10 char, minimum is 6 char
                return NMEA_ERROR; // Error with sentence
            }
            if (strnlen(token, 10) > 6 && token[6] != '.') { // Ensure dot position only if seconds are float
                return NMEA_ERROR; // Error with sentence
            }

            char time_raw[7]; // hhmmss\0
            strncpy(time_raw, token, 6);
            gps_data->time_raw = atoi(time_raw);

            char hours[3] = "00"; // hh\0 = 3 char. Default value because strncpy doesn't add \0 at the end
            strncpy(hours, token, 2);
            gps_data->time.hours = atoi(hours);

            char minutes[3] = "00"; // mm\0 = 3 char
            strncpy(minutes, token+2, 2);
            gps_data->time.minutes = atoi(minutes);

            char seconds[7] = "00.000"; // ss.sss\0 = 7 char
            strncpy(seconds, token+4, 6);
            gps_data->time.seconds = atof(seconds);
        } else if (tok_idx == 2) { // GPS FIX
            // Ensure validity is A or V, else throw error
            if (token[0] == 'A') {
                gps_data->fix = 1; // GPS Fix
            } else if (token[0] == 'V') {
                gps_data->fix = 0; // No GPS Fix
            } else {
                return NMEA_ERROR; // Error with sentence
            }

            if (gps_data->fix == 0) {
                gps_data->latitude = 0;
                gps_data->longitude = 0;
                break; // No fix, so lat/lon are empty
            }
        } else if (tok_idx == 3) { // LATITUDE
            // token = ddmm.mmmmmm (4 to 6 decimals)
            if (strnlen(token, 9) < 9 || token[4] != '.') { //strlen au lieu de strnlen puisqu'on utilise déjà strtok, donc il y a assurément un \0 à la fin?
                return NMEA_ERROR; // Error with sentence
            }

            char degrees[3] = "00"; // dd\0 = 3 char. Default value because strncpy doesn't add \0 at the end
            strncpy(degrees, token, 2);

            char minutes[10] = "00.000000"; // mm.mmmmmm\0 = 10 char max
            strncpy(minutes, token+2, 9);

            gps_data->latitude = atoi(degrees) + atof(minutes) / 60; // Degrees Minutes to Degrees Decimal conversion

        } else if (tok_idx == 4) { // LATITUDE INDICATOR
            int8_t sign;

            // Ensure token is N or S, else throw error
            if (token[0] == 'N') {
                sign = 1;
            } else if (token[0] == 'S') {
                sign = -1;
            } else {
                return NMEA_ERROR; // Error with sentence
            }

            gps_data->latitude *= sign; // Change the sign of latitude depending on indicator

        } else if (tok_idx == 5) { // LONGITUDE
            // token = dddmm.mmmmmm (4 to 6 decimals)
            if (strnlen(token, 10) < 10 || token[5] != '.') { //strlen au lieu de strnlen puisqu'on utilise déjà strtok, donc il y a assurément un \0 à la fin?
                return NMEA_ERROR; // Error with sentence
            }

            char degrees[4] = "000"; // ddd\0 = 4 char. Default value because strncpy doesn't add \0 at the end
            strncpy(degrees, token, 3);

            char minutes[10] = "00.000000"; // mm.mmmmmm\0 = 10 char max
            strncpy(minutes, token+3, 9);

            gps_data->longitude = atoi(degrees) + atof(minutes) / 60; // Degrees Minutes to Degrees Decimal conversion

        } else if (tok_idx == 6) { // LONGITUDE INDICATOR
            int8_t sign;

            // Ensure token is E or W, else throw error
            if (token[0] == 'E') {
                sign = 1;
            } else if (token[0] == 'W') {
                sign = -1;
            } else {
                return NMEA_ERROR; // Error with sentence
            }

            gps_data->longitude *= sign;
        }

        token = strtok(NULL, ",");
        tok_idx++;
    }

    free(copy);
    return NMEA_OK;
}

