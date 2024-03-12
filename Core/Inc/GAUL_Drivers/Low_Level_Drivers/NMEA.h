/*
 * NMEA.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_


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

	// Format : 3150.7856,N,11711.9479,E,102243

	//Real data
	uint16_t latitudeHigh;	// Entier
	uint16_t latitudeLow;	// Decimal
	uint16_t latIndicator;
	uint16_t longitudeHigh;
	uint16_t longitudeLow;
	uint16_t longIndicator;

	uint32_t utcTime;

} NMEA;

// Functions
void NMEA_Convert_DataFormat(NMEA *dev, char *nmeaSentence);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_ */
