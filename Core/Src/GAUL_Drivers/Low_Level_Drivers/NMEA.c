/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"

int NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data) {

    char *token;
    // Créer une copie de la trame
    char *copy = strdup(nmea_sentence);
    if (!copy) {
        return -1;
    }

    // Type Trame (ignore)
    token = strtok(copy, ",");
    if (token == NULL || strcmp(token, "$GPRMC") != 0) {
        free(copy);
        return -1;
    }

    char *fields[8] = {gps_data->time, NULL, gps_data->latitude, NULL, gps_data->longitude, NULL, gps_data->speed_knots, gps_data->track_angle};

	int i = 0;
	while ((token = strtok(NULL, ",")) != NULL && i < 8) {
		if (fields[i] != NULL) {
			strncpy(fields[i], token, sizeof(fields[i]) - 1);
		}
		i++;
	}
	// Indicateur de validite
	gps_data->data_valid = fields[1][0];
	// Indicateur de latitude
	gps_data->latitude_indicator = fields[3][0];
	// Indicateur longitude
	gps_data->longitude_indicator = fields[5][0];

	free(copy);
	return 0; // Succès du décodage
}

