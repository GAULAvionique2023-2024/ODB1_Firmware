/*
 * USART_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: gagnon
 *
 *  Edited on: Aug 11, 2024
 *      Autor: mathouqc
 */

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"

#define TIMEOUT 100000  // Max iterations (Not in ms !!!)

/**
 * Initialise USART peripheral with default settings
 */
void USART_Init(USART_TypeDef *USARTx, uint16_t baudrate, uint16_t frequency_MHz) {

    if (USARTx == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN; // Enable USART1 clock

        Init_GPIO(GPIOB, 6, OUT50, O_AF_PP); // TX (B6)
        Init_GPIO(GPIOB, 7, IN, I_PP); // RX (B7)
    } else if (USARTx == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock

        Init_GPIO(GPIOA, 2, OUT50, O_AF_PP); // TX (A2)
        Init_GPIO(GPIOA, 3, IN, I_PP); // RX (A3)
    } else if (USARTx == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable USART3 clock

        Init_GPIO(GPIOB, 10, OUT50, O_AF_PP); // TX (B10)
        Init_GPIO(GPIOB, 11, IN, I_PP); // RX (B11)
    }

    USART3->CR1 |= USART_CR1_UE; // Activer USART (0x14)
	USART3->CR1 |= USART_CR1_TE; // Activer la transmission
	USART3->CR1 |= USART_CR1_RE; // Activer la réception
	USART3->BRR = (uint16_t)((frequency_MHz * 1000000 + (baudrate / 2)) / baudrate);
}

/**
 * Transmit data with USART in polling/blocking mode
 *
 * @retval 0 OK
 * @retval -1 TIMEOUT ERROR
 */
int8_t USART_TX(USART_TypeDef *USARTx, uint8_t *data, uint16_t size) {

    uint32_t timeout = TIMEOUT;
    while (size--) {
        // Wait until Transmit Data Register is Empty (register is ready to accept new data)
        while (!(USARTx->SR & USART_SR_TXE)) {
            if (--timeout == 0) {
                return -1;
            }
        }
        // Write the byte of data to send
        USARTx->DR = *data++;
    }

    timeout = TIMEOUT;
    // Wait until Transmission Complete
    while (!(USARTx->SR & USART_SR_TC)) {
        if (--timeout == 0) {
            return -1;
        }
    }
    // Clear Data Register (possibly not necessary)
    (void)USARTx->DR;
    return 0;
}

/**
 * Receive data from USART in polling/blocking mode
 *
 * @retval 0 OK
 * @retval -1 TIMEOUT ERROR
 */
int8_t USART_RX(USART_TypeDef *USARTx, uint8_t *data, uint16_t size) {

    uint32_t timeout = TIMEOUT;
    while (size--) {
        // Wait until Transmit Data Register is Empty to avoid overwriting
        // Data Register (DR) while transmission. (possibly not necessary)
        while (!(USARTx->SR & USART_SR_TXE)) {
            if (--timeout == 0) {
                return -1;
            }
        }
        // Full the Data Register. (possibly not necessary)
        USARTx->DR = 0xFF;

        timeout = TIMEOUT;
        // Wait until Read Data Register is Not Empty (byte of data is available)
        while (!(USARTx->SR & USART_SR_RXNE)) {
            if (--timeout == 0) {
                return -1;
            }
        }
        // Read the byte of data
        *data++ = USARTx->DR;
    }
    return 0;
}
