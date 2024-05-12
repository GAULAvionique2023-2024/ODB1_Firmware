/*
 * UART_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "main.h"


void USART_Init(unsigned short usart)
{
    if(usart == 1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        Init_GPIO(PB, 6, OUT50, O_AF_PP); // TX
        Init_GPIO(PB, 7, IN, I_PP); // RX

        USART1->CR1 |= USART_CR1_UE; // Activer USART (0x0C)
        USART1->CR1 |= USART_CR1_TE; // Activer la transmission
        USART1->CR1 |= USART_CR1_RE; // Activer la réception
        // Activation des interruptions globales pour USART1
        NVIC_EnableIRQ(USART1_IRQn);
    }
    else if(usart == 2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

        Init_GPIO(PA, 2, OUT50, O_AF_PP); // TX
        Init_GPIO(PA, 3, IN, I_PP); // RX

        USART2->CR1 |= USART_CR1_UE; // Activer USART (0x10)
        USART2->CR1 |= USART_CR1_TE; // Activer la transmission
        USART2->CR1 |= USART_CR1_RE; // Activer la réception
        // Activation des interruptions globales pour USART2
        NVIC_EnableIRQ(USART2_IRQn);
    }
    else if(usart == 3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN ;

        Init_GPIO(PB, 10, OUT50, O_AF_PP); // TX
        Init_GPIO(PB, 11, IN, I_PP); // RX

        USART3->CR1 |= USART_CR1_UE; // Activer USART (0x14)
        USART3->CR1 |= USART_CR1_TE; // Activer la transmission
        USART3->CR1 |= USART_CR1_RE; // Activer la réception
        // Activation des interruptions globales pour USART3
        NVIC_EnableIRQ(USART3_IRQn);
    }
}

void USART_TX(unsigned short usart, const uint8_t *data, int size) {

    if (usart == 1) {
        for (int i = 0; i < size; i++) {
            while (!(USART1->SR & USART_SR_TXE));
            USART1->DR = data[i];
            while (!(USART1->SR & USART_SR_TC));
        }
    } else if (usart == 2) {
        for (int i = 0; i < size; i++) {
            while (!(USART2->SR & USART_SR_TXE));
            USART2->DR = data[i];
            while (!(USART2->SR & USART_SR_TC));
        }
    } else if (usart == 3) {
        for (int i = 0; i < size; i++) {
            while (!(USART3->SR & USART_SR_TXE));
            USART3->DR = data[i];
            while (!(USART3->SR & USART_SR_TC));
        }
    }
}

void USART_RX(unsigned short usart, uint8_t *data, int size) {

    if (usart == 1) {
        for (int i = 0; i < size; i++) {
            while (!(USART1->SR & USART_SR_RXNE));
            data[i] = USART1->DR;
        }
    } else if (usart == 2) {
        for (int i = 0; i < size; i++) {
            while (!(USART2->SR & USART_SR_RXNE));
            data[i] = USART2->DR;
        }
    } else if (usart == 3) {
        for (int i = 0; i < size; i++) {
            while (!(USART3->SR & USART_SR_RXNE));
            data[i] = USART3->DR;
        }
    }
}
