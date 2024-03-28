/*
 * L76LM33.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_L76LM33_H_
#define INC_GAUL_DRIVERS_L76LM33_H_

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"

#include <string.h>

// Types commandes PMTK
typedef enum {

	NMEA_STARTUP_INDICATOR,		// Permet de recevoir un message a l'activation du GPS
	NMEA_SETRMS,				// Set le type de message NMEA a RMS
	NMEA_RESET, 				// Reset le type de message NMEA configure
	NMEA_START_SEARCHSATELLITE,	// Active la recherche de GPS seulement
	NMEA_NAVMODE,				// 0 : Normal (10000m) ; 2 : Aviation +acc (10000m) ; 3 : Ballon +hauteur (80000m) ; 4 : Stationary

} NMEA_PMTKCommands;

typedef struct {

	char *command;

} NMEA_PMTKCommands_TypeDef;


typedef struct {


	//Ajouter "+/-" char
	uint8_t latitude;
	uint8_t latIndicator;
	uint8_t longitude;
	uint8_t longIndicator;

	uint8_t speed;

	// UTC Time
	uint8_t hTime;
	uint8_t mTime;
	uint8_t sTime;

} L76LM33;



// Initialisation
void L76LM33_Init(void);
void L76LM33_Reset(void);
void L76LM33_ParseData(void);

void RFD900_Transmit_GPSTX(NMEA_PMTKCommands command);
void RFD900_Receive_GPSRX(L76LM33 *devGPS); // Envoyer des commandes au GPS

#endif /* INC_GAUL_DRIVERS_L76LM33_H_ */
