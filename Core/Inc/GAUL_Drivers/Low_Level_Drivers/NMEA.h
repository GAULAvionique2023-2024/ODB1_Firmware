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
#include <stdint.h>
#include <ctype.h>


// NMEA est en ASCII
// PMTK commandes de contrôle

/* --RMS
 * $GPRMC,140146.000,A,3150.863861,N,11711.928739,E,0.00,183.85,211019,,,A,V*13<CR><LF>
 */

// Initialisation
void NMEA_Reset(void);

// Functions
void NMEA_ParseData(uint8_t *buffer);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_ */
