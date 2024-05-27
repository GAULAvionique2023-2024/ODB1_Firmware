/*
 * L76LM33.c
 *
 *  Created on: Mar 14, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/L76LM33.h"

/*
NMEA_STARTUP_INDICATOR = "$PMTK011,MTKGPS*08<CR><LF>",		// Permet de recevoir un message a l'activation du GPS
NMEA_SETRMS = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35<CR><LF>",				// Set le type de message NMEA a RMS
NMEA_RESET = "$PMTK314,-1*04<CR><LF>", 				// Reset le type de message NMEA configure
NMEA_GPSSEARCHONLY = "$PMTK353,1,0,0,0,0*2A<CR><LF>",	// Active la recherche de GPS seulement
NMEA_NAVMODE = "PMTK886,2*2A<CR><LF>",				// 0 : Normal (10000m) ; 2 : Aviation +acc (10000m) ; 3 : Ballon +hauteur (80000m) ; 4 : Stationary
*/

uint8_t L76LM33_Init (void) {

	char PROTOCOL_SETRMS[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n";
	char PROTOCOL_GPSSEARCHONLY[] = "$PMTK353,1,0,0,0,0*2A\r\n";
	char PROTOCOL_NAVMODE[] = "PMTK886,2*2A\r\n";
	L76LM33_SendCommand(PROTOCOL_SETRMS);
	L76LM33_SendCommand(PROTOCOL_GPSSEARCHONLY);
	L76LM33_SendCommand(PROTOCOL_NAVMODE);

	return 1; // OK
}

uint8_t L76LM33_SendCommand(char *command) {

	USART_TX(GPS_USART_PORT, (uint8_t*)command, strlen(command));
	return 1; // OK
}
