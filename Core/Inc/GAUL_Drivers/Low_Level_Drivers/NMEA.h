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

// NMEA est en ASCII [57]
// PMTK commandes AT de contrôle

// --GLL
/* $				GNGLL	,	3150.7856,N		,	11711.9479,E	,		102243.000	,	A		,	D				*				4B			<CR><LF>
 *
 * Start Message	Type		Latitude,Indicator	Longitude,Indicator		UTCTime			Data Valid	Position Mode	End Data field	CheckSum	End Message
 * 					GLL			ddmm.mmmmm	N/S		dddmm.mmmmm	 E/W		hhmmss.sss		V=Invalid	N=No fix						CRC
 * 								degree / minute		""""									A=Valid		A=Auto GNSS
 * 																										D=Diff GNSS
*/

typedef struct {

	// GPS Data
	int32_t latitude;
	uint8_t latIndicator;
	int32_t longitude;
	uint8_t longIndicator;

	// UTC Time
	int8_t hTime;
	int8_t mTime;
	int8_t sTime;

} NMEA_GPSData;					// Variables importantes dans le Data field (entre $ - *)

// Initialisation
void NMEA_Reset(void);

// Functions
void NMEA_ParseData(uint8_t *buffer);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_ */
