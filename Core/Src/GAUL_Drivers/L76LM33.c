/*
 * L76LM33.c
 *
 * L76LM33 is a GNSS module used to get the position (latitude, longitude) of the rocket.
 *
 * This module handles reading UART to receive NMEA sentence, then it parses the sentence
 * using the NMEA module into a structure.
 *
 * Store UART received byte via interrupts into circular buffer using L76LM33_RxCallback()
 * Read circular buffer to find NMEA sentence and parse it using L76LM33_Read()
 *
 *  Created on: May 12, 2024
 *      Author: gagnon
 *
 *  Edited on: Jul 4, 2024
 *      Autor: mathouqc
 */

#include "GAUL_Drivers/L76LM33.h"

/*
 * Source:
 * LG76 Series GNSS Protocol Specification - Section 2.3. PMTK Messages
 *
 * Only output RMC (Recommended Minimum Specific GNSS Sentence) once every one position fix
 * NMEA_RMC[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35<CR><LF>"
 *
 * Set the navigation mode to "Aviation Mode" (for large acceleration movement, altitude of 10'000m max)
 * NMEA_NAVMODE = "PMTK886,2*2A<CR><LF>"
 */

/**
 * Initialize L76LM33 sensor.
 *
 * @param L76_data: pointer to a L76LM33 structure.
 * @param huart: pointer to the GPS HAL UART handler.
 *
 * @retval L76LM33_OK
 * @retval L76LM33_ERROR
 */
int8_t L76LM33_Init(L76LM33 *L76_data, UART_HandleTypeDef *huart) {
    // Set UART handler
    L76_data->huart = huart;

    // Initialize circular buffer
    ring_buffer_init(&(L76_data->UART_Buffer), L76_data->UART_Buffer_arr, L76LM33_BUFFER_SIZES);

    // Receive UART data with interrupts
    if (HAL_UART_Receive_IT(L76_data->huart, &(L76_data->received_byte), 1) != HAL_OK) {
        L76_data->state = 0; // Bad state
        return L76LM33_ERROR; // Error with UART
    }

    // Only output GPRMC sentence
    char NMEA_RMC[] = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*35\r\n";
    if (L76LM33_Send_Command(L76_data, NMEA_RMC, sizeof(NMEA_RMC)) != L76LM33_OK) {
        L76_data->state = 0; // Bad state
        return L76LM33_ERROR; // Error with UART
    }
    // Set navigation mode
    char NMEA_NAVMODE[] = "PMTK886,2*2A\r\n";
    if (L76LM33_Send_Command(L76_data, NMEA_NAVMODE, sizeof(NMEA_NAVMODE)) != L76LM33_OK) {
        L76_data->state = 0; // Bad state
        return L76LM33_ERROR; // Error with UART
    }

    L76_data->state = 1; // Good state
    return L76LM33_OK; // OK
}

/**
 * Callback called on incoming UART data. It is called when HAL_UART_RxCpltCallback is called in main.c.
 * Add received byte to UART circular buffer.
 *
 * @param L76_data: pointer to a L76LM33 structure.
 * @param huart: pointer to a HAL UART handler triggering the callback.
 */
void L76LM33_RxCallback(L76LM33 *L76_data, UART_HandleTypeDef *huart) {
    if (huart->Instance == L76_data->huart->Instance) {
        // Add data to circular buffer
        ring_buffer_queue(&(L76_data->UART_Buffer), L76_data->received_byte);
        if (L76_data->received_byte == '\n') {
            L76_data->new_line_flag = 1;
        }
        // Receive UART data with interrupts
        HAL_UART_Receive_IT(L76_data->huart, &(L76_data->received_byte), 1);
    }
}

/**
 * Read and parse a NMEA GPRMC sentence into data structure. Call this function
 * frequently to have the latest GPS data available.
 *
 * Takes 0.3ms to complete when buffer is full
 *
 * @param L76_data: pointer to a L76LM33 structure to update.
 *
 * @retval L76LM33_OK
 * @retval L76LM33_ERROR
 * @retval L76LM33_EMPTY_BUFF Empty UART buffer, struct unchanged
 *
 */
int8_t L76LM33_Read(L76LM33 *L76_data) {
    // Read sentence
    int8_t valid = L76LM33_Read_Sentence(L76_data);
    if (valid == L76LM33_EMPTY_BUFF) {
        // Empty buffer, don't change structure
        return L76LM33_EMPTY_BUFF;
    } else if (valid != L76LM33_OK) {
        L76_data->state = 0; // Bad state
        return L76LM33_ERROR; // Error
    }

    // Validate sentence ID is RMC
    if (NMEA_ValidateRMC(L76_data->NMEA_Buffer) != NMEA_OK) {
        L76_data->state = 0; // Bad state
        return L76LM33_ERROR; // Error, sentence ID is not RMC
    }

    // Parse NMEA RMC sentence to local structure
    if (NMEA_ParseRMC(&(L76_data->gps_data), L76_data->NMEA_Buffer) != NMEA_OK) {
        L76_data->state = 0; // Bad state
        return L76LM33_ERROR;
    }

    L76_data->state = 1; // Good status (no error)
    return L76LM33_OK; // OK
}

/**
 * Read NMEA sentence from UART circular buffer into a NMEA buffer.
 *
 * Takes 0.24ms to complete when buffer is full
 *
 * @param L76_data: pointer to a L76LM33 structure.
 *
 * @retval L76LM33_OK
 * @retval L76LM33_ERROR cannot find starting or ending character.
 * @retval L76LM33_EMPTY_BUFF empty UART buffer
 *
 */
int8_t L76LM33_Read_Sentence(L76LM33 *L76_data) {
    if (L76_data->new_line_flag == 0) {
        return L76LM33_EMPTY_BUFF; // Error, empty UART circular buffer
    }

    // Reset flag
    L76_data->new_line_flag = 0;

    // Clear NMEA buffer
    for (int16_t i = 0; i < sizeof(L76_data->NMEA_Buffer); i++) {
        L76_data->NMEA_Buffer[i] = 0;
    }

    // Variable to store character from UART buffer
    char c;

    // Try to find '$' in 100 iterations
    for (uint16_t i = 0; i < 100; i++) {
        // Read character from UART buffer
        if (ring_buffer_dequeue(&(L76_data->UART_Buffer), &c) == 0) {
            return L76LM33_EMPTY_BUFF; // Error, empty buffer
        }

        if (c == '$') {
            // Set starting character in NMEA buffer
            L76_data->NMEA_Buffer[0] = '$';

            break; // Found starting characters
        }
    }

    if (c != '$') {
        return L76LM33_ERROR; // Error, cannot find starting character in 100 iterations
    }


    // Read into NMEA buffer until ending character is found
    for (uint16_t i = 1; i < sizeof(L76_data->NMEA_Buffer); i++) {
        // Read character from UART buffer
        if (ring_buffer_dequeue(&(L76_data->UART_Buffer), &c) == 0) {
            return L76LM33_EMPTY_BUFF; // Error, empty buffer
        }

        // Add character to NMEA buffer
        L76_data->NMEA_Buffer[i] = c;

        if (c == '\n') {
            break; // Found ending character
        }
    }

    if (c != '\n') {
        return L76LM33_ERROR; // Error, cannot find '\n'
    }

    return L76LM33_OK;
}

/**
 * Send array of character to L76LM33 using UART HAL functions.
 *
 * @param L76_data: pointer to a L76LM33 structure.
 * @param command[]: array of character to send.
 * @param size: size of the data to send.
 *
 * @retval L76LM33_OK
 * @retval L76LM33_ERROR
 */
int8_t L76LM33_Send_Command(L76LM33 *L76_data, char command[], uint8_t size) {
    if (command == NULL) {
        return L76LM33_ERROR; // Error
    }

    if (HAL_UART_Transmit(L76_data->huart, (uint8_t *)command, size, L76LM33_UART_TIMEOUT) != HAL_OK) {
        return L76LM33_ERROR; // Error with UART
    }

    return L76LM33_OK; // OK
}
