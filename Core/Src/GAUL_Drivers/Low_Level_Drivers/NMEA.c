/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Structures for parsed data
NMEA_GPSData *GPSData;                   // Data from GNSS sentences
NMEA_PMTK_Commands *PMTKData;            // Data from PMTK sentences


void NMEA_Init(void) {

	memset(GPSData, 0, 7);
	memset(PMTKData, 0, 4);

}

void NMEA_ParseData(uint8_t *buffer) {

	int size = 0;
	char *ptr = buffer;

	memcpy(GPSData->type, ptr, 6);
	ptr += 7;

	// Longueur du data Field
	if(GPSData->type == 0x244750524D43) //$GPRMC
	{
		//size =
	}
}
