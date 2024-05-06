/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"

// Lors de la réception des paquets, on devra convertir les bytes en hex et les hex en char pour revoir la forme initiale du message NMEA
// La methode permettra de réduire  1/2 la quatité de bytes a transmettre

uint16_t charToHex(char *buffer) {

	// Convertir char -> hex
	int tail = strlen(buffer) - 11;


	for(int i = 0; i < tail; i++)
	{
		// Premier char -> $ (delim start)
		if(isdigit(buffer[i]))
		{

		}
		else if(isalpha(buffer[i]))
		{

		}
	}
}

uint8_t hexToByte(char *buffer) {

	// Convertir hex -> byte
}

void NMEA_Reset(void) {

	//memset(GPSData, 0, 7);

	// Reset delim
}

void GetStatusNMEA(char *buffer) {

	char *start = buffer;
}

// Mettre données dans un tableau afin de les transmettre au RFD900x (Modifier buffer ou retourner tableau?)
void NMEA_ParseData(uint8_t *buffer) {


}
