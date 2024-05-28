/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/NMEA.h"

uint8_t NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data) {

	// Créer une copie de la trame pour la manipulation
	char *copy = strdup(nmea_sentence);
	if (!copy) {
		return 0;
	}

	char *token;

	// Extraire le type de trame NMEA
	token = strtok(copy, ",");
	if (token == NULL || strcmp(token, "$GPRMC") != 0) {
	    free(copy);
	    return 0;
	}

	// Extraire utc time
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return 0;
	} else {
		strncpy(gps_data->time, token, 11);
		gps_data->time[11] = '\0';
	}


	// Vérifier caractere de validite
	token = strtok(NULL, ",");
	if (token == NULL || strcmp(token, "A") != 0) {
		free(copy);
		return 0;
	}

	// Extraire latitude
	token = strtok(NULL, ",");
	if (token == NULL) {
		strncpy(gps_data->latitude, DEFAULT_LATITUDE, 10);
		return 0;
	} else {
		strncpy(gps_data->latitude, token, 10);
	}
	gps_data->latitude[10] = '\0';

	// Extraire l'indicateur de latitude (N ou S)
	token = strtok(NULL, ",");
	if (token == NULL) {
		strcpy(gps_data->latitude_indicator, DEFAULT_INDICATOR);
		return 0;
	} else {
		gps_data->latitude_indicator = token[0];
	}

	// Extraire longitude
	token = strtok(NULL, ",");
	if (token == NULL) {
		strncpy(gps_data->longitude, DEFAULT_LATITUDE, 11);
		return 0;
	} else {
		strncpy(gps_data->longitude, token, 11);
	}
	gps_data->longitude[11] = '\0';

	// Extraire l'indicateur de longitude (E ou W)
	token = strtok(NULL, ",");
	if (token == NULL) {
		strcpy(gps_data->longitude_indicator, DEFAULT_INDICATOR);
		return 0;
	} else {
		gps_data->longitude_indicator = token[0];
	}

	// Extraire vitesse
	token = strtok(NULL, ",");
	if (token == NULL) {
		strncpy(gps_data->speed_knots, DEFAULT_SPEED, 6);
		return 0;
	} else {
		strncpy(gps_data->speed_knots, token, 6);
	}
	gps_data->speed_knots[6] = '\0';

	// Extraire angle
	token = strtok(NULL, ",");
	if (token == NULL) {
		strncpy(gps_data->track_angle, DEFAULT_ANGLE, 7);
		return 0;
	} else {
		strncpy(gps_data->track_angle, token, 7);
	}
	gps_data->track_angle[7] = '\0';

	free(copy);
	return 1; // OK
}

uint8_t NMEA_ValidTrame(const char *nmea_sentence) {

	uint8_t valid = 1;

	char *copy = strdup(nmea_sentence);
	char *token;
	token = strtok(copy, ",");
	token = strtok(NULL, ",");
	token = strtok(NULL, ",");
	if (token == NULL || strcmp(token, "A") != 0) {
		free(copy);
		valid = 0;
	}

	return valid;
}
/*
uint8_t NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data) {
    if (nmea_sentence == NULL || gps_data == NULL) {
        return 0; // Error: Null pointer
    }

    char type[6];
    int n = sscanf(nmea_sentence, "$%5[^,],%11[^,],%*c,%10[^,],%c,%11[^,],%c,%6[^,],%7[^,]",
                   type,
                   gps_data->time,
                   gps_data->latitude,
                   &gps_data->latitude_indicator,
                   gps_data->longitude,
                   &gps_data->longitude_indicator,
                   gps_data->speed_knots,
                   gps_data->track_angle);

    // Check if the type is "GPRMC" and the required fields are successfully parsed
    if (n == 8 && strcmp(type, "GPRMC") == 0) {
        // Ensure string termination
        gps_data->time[11] = '\0';
        gps_data->latitude[10] = '\0';
        gps_data->longitude[11] = '\0';
        gps_data->speed_knots[6] = '\0';
        gps_data->track_angle[7] = '\0';
        return 1; // Success
    }

    // Fill with default values if parsing failed
    strcpy(gps_data->latitude, DEFAULT_LATITUDE);
    gps_data->latitude_indicator = DEFAULT_INDICATOR[0];
    strcpy(gps_data->longitude, DEFAULT_LONGITUDE);
    gps_data->longitude_indicator = DEFAULT_INDICATOR[0];
    strcpy(gps_data->speed_knots, DEFAULT_SPEED);
    strcpy(gps_data->track_angle, DEFAULT_ANGLE);
    return 0; // Error: Parsing failed
}
*/
