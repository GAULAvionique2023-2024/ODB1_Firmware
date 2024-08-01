/*
 * L76LM33.c
 *
 *  Created on: Mar 14, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/L76LM33.h"

char gps_buffer[BUFFER_SIZE];
uint16_t buffer_index = 0;

uint8_t L76LM33_Init(L76LM33 *devL76L) {

    char PROTOCOL_SETRMS[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n";
    char PROTOCOL_GPSSEARCHONLY[] = "$PMTK353,1,0,0,0,0*2A\r\n";
    char PROTOCOL_NAVMODE[] = "PMTK886,2*2A\r\n";
    L76LM33_SendCommand(devL76L, PROTOCOL_SETRMS);
    L76LM33_SendCommand(devL76L, PROTOCOL_GPSSEARCHONLY);
    L76LM33_SendCommand(devL76L, PROTOCOL_NAVMODE);

    return 1; // OK
}

uint8_t L76LM33_SendCommand(L76LM33 *devL76L, char *command) {

    if (command == NULL) {
        return 0; // Error
    }
    USART_TX(devL76L->USARTx, (uint8_t*)command, strlen(command));
    return 1; // OK
}

uint8_t L76LM33_Read(L76LM33 *devL76L, char *rx_data, GPS_Data *gps_data) {

    if (rx_data == NULL || gps_data == NULL) {
        return 0; // Error
    }

    uint8_t byte;
    while (1) {
        USART_RX(devL76L->USARTx, &byte, 1);
        if (byte == '$') {
            buffer_index = 0;
        }
        gps_buffer[buffer_index++] = byte;
        if (buffer_index >= BUFFER_SIZE) {
            buffer_index = 0;
        }
        if (byte == '\n' && buffer_index > 0) {
            gps_buffer[buffer_index] = '\0';
            strcpy(rx_data, gps_buffer);
            buffer_index = 0;
            if (NMEA_ValidTrame(rx_data)) {
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
        }
    }

    return 0; // No complete sentence found
}
