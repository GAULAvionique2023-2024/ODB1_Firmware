/*
 * NMEA_tests.h
 *
 *  Created on: Jul 25, 2024
 *      Author: mathouqc
 */

#ifndef INC_GAUL_DRIVERS_TESTS_NMEA_TESTS_H_
#define INC_GAUL_DRIVERS_TESTS_NMEA_TESTS_H_

#include "GAUL_Drivers/NMEA.h"

void NMEA_TESTS_ValidateRMC_Log();
void NMEA_TESTS_ParseRMC_Log();

void NMEA_TESTS_LogStructure(GPS_Data *gps_data);

#endif /* INC_GAUL_DRIVERS_TESTS_NMEA_TESTS_H_ */
