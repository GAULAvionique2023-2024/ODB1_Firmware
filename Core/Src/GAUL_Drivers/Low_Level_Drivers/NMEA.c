/*
 * NMEA.c
 *
 *  Created on: Mar 12, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/NMEA.h"
#include <string.h>
#include <stdlib.h>


void NMEA_Convert_DataFormat(NMEA *dev, char *nmeaSentence) {

	const char outer_delimiters[] = ",";
	const char inner_delimiters[] = ".";

	char* token;
	char* outer_saveptr = NULL;
	char* inner_saveptr = NULL;

	token = strtok_r(nmeaSentence, outer_delimiters, &outer_saveptr);

	int i = 0;
	while (token != NULL)
	{
		char* inner_token = strtok_r(token, inner_delimiters, &inner_saveptr);

		while (inner_token != NULL)
		{
			if(i == 1)
			{
				dev->latitudeHigh = atoi(inner_token);
			}
			else if(i == 2)
			{
				dev->latitudeLow = atoi(inner_token);
			}
			else if(i == 3)
			{
				char *output;
				dev->latIndicator = strtol(inner_token, &output, 10) + 1;
			}
			else if(i == 4)
			{
				dev->longitudeHigh = atoi(inner_token);
			}
			else if(i == 5)
			{
				dev->longitudeLow = atoi(inner_token);
			}
			else if(i == 6)
			{
				char *output;
				dev->latIndicator = strtol(inner_token, &output, 10) + 1;
			}
			else if(i == 7)
			{
				dev->utcTime = atoi(inner_token);
			}

			inner_token = strtok_r(NULL, inner_delimiters, &inner_saveptr);

			i++;
		}

		token = strtok_r(NULL, outer_delimiters, &outer_saveptr);
	}
}


