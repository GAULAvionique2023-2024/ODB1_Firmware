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

uint8_t L76LM33_Init (unsigned short usart_port) {

	char PROTOCOL_SETRMS[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n";
	char PROTOCOL_GPSSEARCHONLY[] = "$PMTK353,1,0,0,0,0*2A\r\n";
	char PROTOCOL_NAVMODE[] = "PMTK886,2*2A\r\n";
	L76LM33_SendCommand(usart_port, PROTOCOL_SETRMS);
	L76LM33_SendCommand(usart_port, PROTOCOL_GPSSEARCHONLY);
	L76LM33_SendCommand(usart_port, PROTOCOL_NAVMODE);

	return 1; // OK
}

uint8_t L76LM33_SendCommand(unsigned short usart_port, char *command) {

    if (command == NULL) {
        return 0; // Error
    }
    USART_TX(usart_port, (uint8_t*)command, strlen(command));
    return 1; // OK
}

uint8_t L76LM33_Read(unsigned short usart_port, char *rx_data, GPS_Data *gps_data) {

    if (rx_data == NULL || gps_data == NULL) {
        return 0; // Error
    }
    for(int j = 0; j < NMEA_TRAME_RMC_SIZE; j++) {
    	rx_data[j] = "\0";
    }
    USART_RX(usart_port, (uint8_t*)rx_data, NMEA_TRAME_RMC_SIZE);
    printf("NMEA sentence: %s\n", rx_data);
    if (NMEA_Decode_GPRMC(rx_data, gps_data) == 1) {
        printf("Time: %ld\n", gps_data->time);
        printf("Latitude: %ld %c\n", gps_data->latitude, (char)gps_data->latitude_indicator);
        printf("Longitude: %ld %c\n", gps_data->longitude, (char)gps_data->longitude_indicator);
        printf("Speed: %ld\n", gps_data->speed_knots);
        printf("Angle: %ld\n", gps_data->track_angle);
        return 1; // OK
    } else {
        printf(" -> Failed to decode NMEA sentence.\n");
        return 0; // Error
    }
}
