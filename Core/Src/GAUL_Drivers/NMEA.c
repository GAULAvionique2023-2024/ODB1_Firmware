/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/NMEA.h"

// Fonction pour convertir un float en tableau de uint8_t
int32_t NMEA_FloatToBytes(float value) {

    return (int32_t)value;
}

// Fonction pour convertir une chaîne de caractères en float
float NMEA_CharToFloat(char *data) {

    return (float)atof(data);
}

// Fonction pour décoder une trame NMEA GPRMC
uint8_t NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data) {

    if (!nmea_sentence || !gps_data) {
        return 0;
    }

    char *copy = strdup(nmea_sentence);
    if (!copy) {
        return 0;
    }

    char *token;
    int field_index = 0;
    int valid = 1;

    // Valeurs par default
	gps_data->latitude = DEFAULT_LATITUDE;
	gps_data->latitude_indicator = DEFAULT_INDICATOR;
	gps_data->longitude = DEFAULT_LONGITUDE;
	gps_data->longitude_indicator = DEFAULT_INDICATOR;
	gps_data->speed_knots = DEFAULT_SPEED;
	gps_data->track_angle = DEFAULT_ANGLE;

    // Extraire chaque champ de la trame
    token = strtok(copy, ",");
    while (token != NULL && valid) {
        switch (field_index) {
            case 0:
                if (strcmp(token, "$GPRMC") != 0) {
                    valid = 0;
                }
                break;
            case 1:
            	gps_data->time = NMEA_FloatToBytes(NMEA_CharToFloat(token));
                break;
            case 2:
                if (strcmp(token, "A") != 0) {
                    valid = 0;
                }
                break;
            case 3:
            	gps_data->latitude = NMEA_FloatToBytes(NMEA_CharToFloat(token));
                break;
            case 4:
                gps_data->latitude_indicator = (uint8_t)token[0];
                break;
            case 5:
                gps_data->longitude = NMEA_FloatToBytes(NMEA_CharToFloat(token));
                break;
            case 6:
                gps_data->longitude_indicator = (uint8_t)token[0];
                break;
            case 7:
                gps_data->speed_knots = NMEA_FloatToBytes(NMEA_CharToFloat(token));
                break;
            case 8:
                gps_data->track_angle = NMEA_FloatToBytes(NMEA_CharToFloat(token));
                break;
        }
        token = strtok(NULL, ",");
        field_index++;
    }

    free(copy);
    return valid;
}

// Fonction pour vérifier la validité d'une trame NMEA
uint8_t NMEA_ValidTrame(const char *nmea_sentence) {

    char *copy = strdup(nmea_sentence);
    if (!copy) {
        return 0;
    }

    uint8_t valid = 1;
    char *token;

    token = strtok(copy, ",");
    if (token == NULL || strcmp(token, "$GPRMC") != 0) {
        valid = 0;
    } else {
        token = strtok(NULL, ","); // Skip time
        token = strtok(NULL, ","); // Validity indicator
        if (token == NULL || strcmp(token, "A") != 0) {
            valid = 0;
        }
    }

    free(copy);
    return valid;
}

