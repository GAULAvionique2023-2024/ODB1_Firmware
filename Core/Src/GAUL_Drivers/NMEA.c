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
