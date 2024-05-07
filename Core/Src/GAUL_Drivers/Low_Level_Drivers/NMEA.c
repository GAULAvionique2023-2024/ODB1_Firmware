/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"

int NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data) {

	// Créer une copie de la trame pour la manipulation
	char *copy = strdup(nmea_sentence);
	if (!copy) {
		return -1;
	}

	char *token;

	// Extraire le type de trame NMEA
	token = strtok(copy, ",");
	if (token == NULL || strcmp(token, "$GPRMC") != 0) {
		free(copy);
		return -1;
	}

	// Extraire utc time
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	strncpy(gps_data->time, token, 11);

	// Vérifier caractere de validite
	token = strtok(NULL, ",");
	if (token == NULL || strcmp(token, "A") != 0) {
		free(copy);
		return -1;
	}

	// Extraire latitude
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	strncpy(gps_data->latitude, token, 9);
	gps_data->latitude[9] = '\0';

	// Extraire l'indicateur de latitude (N ou S)
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	gps_data->latitude_indicator = token[0];

	// Extraire longitude
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	strncpy(gps_data->longitude, token, 11);
	gps_data->longitude[11] = '\0';

	// Extraire l'indicateur de longitude (E ou W)
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	gps_data->longitude_indicator = token[0];

	// Extraire vitesse
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	strncpy(gps_data->speed_knots, token, 6);

	// Extraire angle
	token = strtok(NULL, ",");
	if (token == NULL) {
		free(copy);
		return -1;
	}
	strncpy(gps_data->track_angle, token, 7);

	free(copy);
	return 0;
}

