/*
 * UART_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#include <GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h>
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"

#define TIMEOUT 200  // Timeout value

void USART_Init(USART_TypeDef *USARTx) {

    if (USARTx == USART1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        Init_GPIO(GPIOB, 6, OUT50, O_AF_PP); // TX
        Init_GPIO(GPIOB, 7, IN, I_PP); // RX

        USART1->CR1 |= USART_CR1_UE; // Activer USART (0x0C)
        USART1->CR1 |= USART_CR1_TE; // Activer la transmission
        USART1->CR1 |= USART_CR1_RE; // Activer la réception
    } else if (USARTx == USART2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

        Init_GPIO(GPIOA, 2, OUT50, O_AF_PP); // TX
        Init_GPIO(GPIOA, 3, IN, I_PP); // RX

        USART2->CR1 |= USART_CR1_UE; // Activer USART (0x10)
        USART2->CR1 |= USART_CR1_TE; // Activer la transmission
        USART2->CR1 |= USART_CR1_RE; // Activer la réception
    } else if (USARTx == USART3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

        Init_GPIO(GPIOB, 10, OUT50, O_AF_PP); // TX
        Init_GPIO(GPIOB, 11, IN, I_PP); // RX

        USART3->CR1 |= USART_CR1_UE; // Activer USART (0x14)
        USART3->CR1 |= USART_CR1_TE; // Activer la transmission
        USART3->CR1 |= USART_CR1_RE; // Activer la réception
    }
}

int USART_TX(USART_TypeDef *USARTx, uint8_t *data, int size) {

    uint32_t timeout = TIMEOUT;
    while (size--) {
        while (!(USARTx->SR & USART_SR_TXE)) {
            if (--timeout == 0) {
                return -1;
            }
        }
        USARTx->DR = *data++;
    }

    timeout = TIMEOUT;
    while (!(USARTx->SR & USART_SR_TC)) {
        if (--timeout == 0) {
            return -1;
        }
    }
    (void)USARTx->DR;
    return 0;
}

int USART_RX(USART_TypeDef *USARTx, uint8_t *data, int size) {

    uint32_t timeout = TIMEOUT;
    while (size--) {
        while (!(USARTx->SR & USART_SR_TXE)) {
            if (--timeout == 0) {
                return -1;
            }
        }
        USARTx->DR = 0xFF;

        timeout = TIMEOUT;
        while (!(USARTx->SR & USART_SR_RXNE)) {
            if (--timeout == 0) {
                return -1;
            }
        }
        *data++ = USARTx->DR;
    }
    return 0;
}
