/*
 * NMEA.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_NMEA_H
#define INC_GAUL_DRIVERS_NMEA_H

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#define NMEA_TRAME_RMC_SIZE 68

// Définition des valeurs par défaut en tant que macros
#define DEFAULT_LATITUDE 	0x00000000
#define DEFAULT_LONGITUDE 	0x00000000
#define DEFAULT_SPEED 		0x00000000
#define DEFAULT_ANGLE 		0x00000000
#define DEFAULT_INDICATOR 	56 // "V"

typedef struct {
    int32_t time;         		// Heure en bytes
    int32_t latitude;     		// Latitude en bytes
    uint8_t latitude_indicator; 	// Indicateur de latitude (N ou S)
    int32_t longitude;			// Longitude en bytes
    uint8_t longitude_indicator;	// Indicateur de longitude (E ou W)
    int32_t speed_knots;			// Vitesse sur le fond en noeuds en bytes
    int32_t track_angle;			// Route sur le fond en degres en bytes
} GPS_Data;

uint8_t NMEA_Decode_GPRMC(const char *nmea_sentence, GPS_Data *gps_data);
uint8_t NMEA_ValidTrame(const char *nmea_sentence);

#endif /* INC_GAUL_DRIVERS_NMEA_H */
