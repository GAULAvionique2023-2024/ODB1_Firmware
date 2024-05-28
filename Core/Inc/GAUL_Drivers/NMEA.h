/*
 * NMEA.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <stdint.h>

#define NMEA_TRAME_RMC_SIZE 68

// Définition des valeurs par défaut en tant que macros
#define DEFAULT_LATITUDE 	"00000.000"
#define DEFAULT_LONGITUDE 	"000000.000"
#define DEFAULT_SPEED 		"000.00"
#define DEFAULT_ANGLE 		"0000.00"
#define DEFAULT_INDICATOR 	"V"

/* --RMC
 * $GPRMC,140146.000,A,3150.863861,N,11711.928739,E,0.00,183.85,211019,,,A,V*13<CR><LF>
 */

typedef struct {
    char 	time[12]; // Heure (format HHMMSS.SSS)
    char  	latitude[11]; // Latitude (format dddmm.mmmm)
    char 	latitude_indicator; // Indicateur de latitude (N ou S)
    char  	longitude[12]; // Longitude (format dddmm.mmmm)
    char 	longitude_indicator; // Indicateur de longitude (E ou W)
    char  	speed_knots[7]; // Vitesse sur le fond en noeuds
    char  	track_angle[8]; // Route sur le fond en degres
} GPS_Data;


uint8_t NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data);
uint8_t NMEA_ValidTrame(const char * nmea_sentence);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_ */
