/*
 * L76LM33.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_L76LM33_H_
#define INC_GAUL_DRIVERS_L76LM33_H_

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"

#include <string.h>

// Types commandes PMTK
typedef enum {

	NMEA_STARTUP_INDICATOR,		// Permet de recevoir un message a l'activation du GPS
	NMEA_SETRMSGLL,	// Set le type de message NMEA a GLL + RMS (speed)
	NMEA_RESET, 	// Reset le type de message NMEA configure
	NMEA_START_SEARCHSATELLITE,	// Active la recherche de GPS seulement
	NMEA_NAVMODE,		// 0 : Normal (10000m) ; 2 : Aviation +acc (10000m) ; 3 : Ballon +hauteur (80000m) ; 4 : Stationary

} NMEA_PMTKCommands;

typedef struct {

	char *command;

} NMEA_PMTKCommands_TypeDef;


typedef struct {

	char *message;

} L76LM33;

// Initialisation
void L76LM33_Init(NMEA_PMTKCommands command);

void RFD900_Transmit_GPSTX(L76LM33 *devGPS);
void RFD900_Receive_GPSRX(L76LM33 *devGPS); // Envoyer des commandes au GPS

#endif /* INC_GAUL_DRIVERS_L76LM33_H_ */
