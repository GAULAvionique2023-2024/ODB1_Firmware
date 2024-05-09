/*
 * L76LM33.c
 *
 *  Created on: Mar 14, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/L76LM33.h"
#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include <string.h>

/*
NMEA_STARTUP_INDICATOR = "$PMTK011,MTKGPS*08<CR><LF>",		// Permet de recevoir un message a l'activation du GPS
NMEA_SETRMS = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35<CR><LF>",				// Set le type de message NMEA a RMS
NMEA_RESET = "$PMTK314,-1*04<CR><LF>", 				// Reset le type de message NMEA configure
NMEA_GPSSEARCHONLY = "$PMTK353,1,0,0,0,0*2A<CR><LF>",	// Active la recherche de GPS seulement
NMEA_NAVMODE = "PMTK886,2*2A<CR><LF>",				// 0 : Normal (10000m) ; 2 : Aviation +acc (10000m) ; 3 : Ballon +hauteur (80000m) ; 4 : Stationary
*/

void L76LM33_Init () {

	char PROTOCOL_SETRMS[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35<CR><LF>";
	char PROTOCOL_GPSSEARCHONLY[] = "$PMTK353,1,0,0,0,0*2A<CR><LF>";
	char PROTOCOL_NAVMODE[] = "PMTK886,2*2A<CR><LF>";
	USART_TX(GPS_USART_PORT, (uint8_t*)PROTOCOL_SETRMS, strlen(PROTOCOL_SETRMS));
	USART_TX(GPS_USART_PORT, (uint8_t*)PROTOCOL_GPSSEARCHONLY, strlen(PROTOCOL_GPSSEARCHONLY));
	USART_TX(GPS_USART_PORT, (uint8_t*)PROTOCOL_NAVMODE, strlen(PROTOCOL_NAVMODE));
}

// Refaire ne fonctionne pas
void L76LM33_Read() {
/*
	char nmea_sentence[L76LM33_RX_BUFFER]; // Verifier si uint8_t ou char
	USART_RX(GPS_USART_PORT, nmea_sentence, L76LM33_RX_BUFFER);
	printf("NMEA Sentence receive: %s/n", nmea_sentence);

	GPS_Data gps_data;
	NMEA_Decode_GPRMC(nmea_sentence, &gps_data);
*/
}
