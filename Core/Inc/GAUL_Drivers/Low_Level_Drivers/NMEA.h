/*
 * NMEA.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>


// NMEA est en ASCII
// PMTK commandes de contrôle

/* --RMC
 * $GPRMC,140146.000,A,3150.863861,N,11711.928739,E,0.00,183.85,211019,,,A,V*13<CR><LF>
 */

typedef struct {

    char 	time[12]; // Heure (format HHMMSS.SSS)

    char  	latitude[10]; // Latitude (format dddmm.mmmm)
    char 	latitude_indicator; // Indicateur de latitude (N ou S)
    char  	longitude[12]; // Longitude (format dddmm.mmmm)
    char 	longitude_indicator; // Indicateur de longitude (E ou W)

    char  	speed_knots[6]; // Vitesse sur le fond en noeuds
    char  	track_angle[7]; // Route sur le fond en degres
} GPS_Data;

int NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_ */
