/*
 * L76LM33.c
 *
 *  Created on: Mar 14, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/L76LM33.h"
#include <string.h>

static const NMEA_PMTKCommands_TypeDef PMTKCommandsInit[] = {

    "$PMTK011,MTKGPS*08<CR><LF>",
	"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35<CR><LF>",
    "$PMTK314,-1*04<CR><LF>",
	"$PMTK353,1,0,0,0,0*2A<CR><LF>",
	"$PMTK886,2*2A<CR><LF>",
};

void L76LM33_Init() {

	USART_TX(GPS_USART_PORT, (uint8_t*)NMEA_STARTUP_INDICATOR, 26);
	USART_TX(GPS_USART_PORT, (uint8_t*)NMEA_SETRMS, 63);
	USART_TX(GPS_USART_PORT, (uint8_t*)NMEA_START_SEARCHSATELLITE, 29);
	USART_TX(GPS_USART_PORT, (uint8_t*)NMEA_NAVMODE, 21);
}

void L76LM33_Reset(void) {

	USART_TX(GPS_USART_PORT, (uint8_t*)NMEA_RESET, 22);
}

void L76LM33_Transmit_GPS_TX(NMEA_PMTKCommands command) {

	// USART_TX(GPS_USART_PORT, (uint8_t*)command, strlen(command));
}

void L76LM33_Receive_GPSRX(L76LM33 *devGPS) {

	// Convertir avec NMEA
}
