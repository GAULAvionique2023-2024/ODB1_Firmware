/*
 * L76LM33.h
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_L76LM33_H_
#define INC_GAUL_DRIVERS_L76LM33_H_


typedef struct {

	// $GNGLL,3150.7856,N,11711.9479,E,102243.000,A,D*4B<CR><LF> (57 bytes)
	// Message a envoyer : 3150 7856 N 11711 9479 E 102243 (25 bytes)
	// Format : 		   uint8 des chiffre hex donc doit reconvertir byte en hex et convertir hex en char

	//Real data
	int16_t latitudeHigh;
	int16_t latitudeLow;
	uint8_t latIndicator;
	int16_t longitudeHigh;
	int16_t longitudeLow;
	uint8_t longIndicator;

	uint8_t hTime;
	uint8_t mTime;
	uint8_t sTime;

} L76LM33;

void L76LM33_Init(void);

#endif /* INC_GAUL_DRIVERS_L76LM33_H_ */
