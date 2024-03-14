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

	//Type
	uint8_t type;

	// GPS Data
	int32_t latitude;
	uint8_t latIndicator;
	int32_t longitude;
	uint8_t longIndicator;

	// UTC Time
	int8_t hTime;
	int8_t mTime;
	int8_t sTime;

} NMEA_GPSData;

typedef struct {

	uint8_t  PMTK_BOOT;           // 1 when "$PMTK011,MTKGPS*08" sentence parsed
	uint8_t  PMTK010;             // Last parsed $PMTK010 sentence:
	                              //   0 = unknown
	                              //   1 = startup
	                              //   2 = notification for the host aiding EPO
	                              //   3 = notification for the transition to normal mode is successfully done
	uint16_t PMTK001_CMD;         // CMD field from last parsed $PMTK001 sentence
	uint8_t  PMTK001_FLAG;        // FLAG field from last parsed $PMTK001 sentence:
	                              //   0 = invalid packet
	                              //   1 = unsupported packet type
	                              //   2 = valid packet, but action failed
	                              //   3 = valid packet, action succeeded
} NMEA_PMTK_Commands;

// Initialisation
void NMEA_Init(void);

// Functions
void NMEA_ParseData(uint8_t *buffer);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_NMEA_H_ */
