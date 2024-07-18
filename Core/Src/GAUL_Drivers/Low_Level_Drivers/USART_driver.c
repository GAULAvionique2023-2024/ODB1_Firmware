/*
 * UART_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "main.h"

#define RX_BUFFER_SIZE 128
#define TX_BUFFER_SIZE 128

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];

void USART1_DMA_Init(void);
void USART2_DMA_Init(void);
void USART3_DMA_Init(void);

void USART_Init(unsigned short usart, uint32_t baudrate) {

    if (usart == 1) {
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        Init_GPIO(PB, 6, OUT50, O_AF_PP); // TX
        Init_GPIO(PB, 7, IN, I_PP); // RX

        USART1->BRR = SystemCoreClock / baudrate; // Configurer le baud rate
        USART1->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Activer USART, TX, RX et RXNE Interrupt
        USART1->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; // Activer la DMA pour TX et RX

        NVIC_EnableIRQ(USART1_IRQn); // Activer l'interruption USART1
        USART1_DMA_Init(); // Initialiser la DMA pour USART1
    } else if (usart == 2) {
        RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        Init_GPIO(PA, 2, OUT50, O_AF_PP); // TX
        Init_GPIO(PA, 3, IN, I_PP); // RX

        USART2->BRR = SystemCoreClock / (2 * baudrate); // Configurer le baud rate (APB1 est divisé par 2)
        USART2->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Activer USART, TX, RX et RXNE Interrupt
        USART2->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; // Activer la DMA pour TX et RX

        NVIC_EnableIRQ(USART2_IRQn); // Activer l'interruption USART2
        USART2_DMA_Init(); // Initialiser la DMA pour USART2
    } else if (usart == 3) {
        RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
        Init_GPIO(PB, 10, OUT50, O_AF_PP); // TX
        Init_GPIO(PB, 11, IN, I_PP); // RX

        USART3->BRR = SystemCoreClock / (2 * baudrate); // Configurer le baud rate (APB1 est divisé par 2)
        USART3->CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE; // Activer USART, TX, RX et RXNE Interrupt
        USART3->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR; // Activer la DMA pour TX et RX

        NVIC_EnableIRQ(USART3_IRQn); // Activer l'interruption USART3
        USART3_DMA_Init(); // Initialiser la DMA pour USART3
    }
}

void USART1_DMA_Init(void) {

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Activer l'horloge DMA1

    // Configuration du canal DMA pour la réception (canal 5)
    DMA1_Channel5->CPAR = (uint32_t)&USART1->DR; // Adresse périphérique
    DMA1_Channel5->CMAR = (uint32_t)rx_buffer; // Adresse mémoire (buffer de réception)
    DMA1_Channel5->CNDTR = RX_BUFFER_SIZE; // Taille du buffer
    DMA1_Channel5->CCR = DMA_CCR_MINC | DMA_CCR_TCIE; // Mode mémoire incrémental, interruption de fin de transfert

    // Configuration du canal DMA pour la transmission (canal 4)
    DMA1_Channel4->CPAR = (uint32_t)&USART1->DR; // Adresse périphérique
    DMA1_Channel4->CMAR = (uint32_t)tx_buffer; // Adresse mémoire (buffer de transmission)
    DMA1_Channel4->CNDTR = TX_BUFFER_SIZE; // Taille du buffer
    DMA1_Channel4->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE; // Mode mémoire incrémental, direction (mémoire à périphérique), interruption de fin de transfert

    // Activer les interruptions DMA
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

void USART2_DMA_Init(void) {

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Activer l'horloge DMA1

    // Configuration du canal DMA pour la réception (canal 6)
    DMA1_Channel6->CPAR = (uint32_t)&USART2->DR; // Adresse périphérique
    DMA1_Channel6->CMAR = (uint32_t)rx_buffer; // Adresse mémoire (buffer de réception)
    DMA1_Channel6->CNDTR = RX_BUFFER_SIZE; // Taille du buffer
    DMA1_Channel6->CCR = DMA_CCR_MINC | DMA_CCR_TCIE; // Mode mémoire incrémental, interruption de fin de transfert

    // Configuration du canal DMA pour la transmission (canal 7)
    DMA1_Channel7->CPAR = (uint32_t)&USART2->DR; // Adresse périphérique
    DMA1_Channel7->CMAR = (uint32_t)tx_buffer; // Adresse mémoire (buffer de transmission)
    DMA1_Channel7->CNDTR = TX_BUFFER_SIZE; // Taille du buffer
    DMA1_Channel7->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE; // Mode mémoire incrémental, direction (mémoire à périphérique), interruption de fin de transfert

    // Activer les interruptions DMA
    NVIC_EnableIRQ(DMA1_Channel6_IRQn);
    NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

void USART3_DMA_Init(void) {

    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Activer l'horloge DMA1

    // Configuration du canal DMA pour la réception (canal 3)
    DMA1_Channel3->CPAR = (uint32_t)&USART3->DR; // Adresse périphérique
    DMA1_Channel3->CMAR = (uint32_t)rx_buffer; // Adresse mémoire (buffer de réception)
    DMA1_Channel3->CNDTR = RX_BUFFER_SIZE; // Taille du buffer
    DMA1_Channel3->CCR = DMA_CCR_MINC | DMA_CCR_TCIE; // Mode mémoire incrémental, interruption de fin de transfert

    // Configuration du canal DMA pour la transmission (canal 2)
    DMA1_Channel2->CPAR = (uint32_t)&USART3->DR; // Adresse périphérique
    DMA1_Channel2->CMAR = (uint32_t)tx_buffer; // Adresse mémoire (buffer de transmission)
    DMA1_Channel2->CNDTR = TX_BUFFER_SIZE; // Taille du buffer
    DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE; // Mode mémoire incrémental, direction (mémoire à périphérique), interruption de fin de transfert

    // Activer les interruptions DMA
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
}

void USART_DMA_TX(unsigned short usart, const uint8_t *data, int size) {

    if (usart == 1) {
        memcpy(tx_buffer, data, size); // Copier les données dans le buffer de transmission
        DMA1_Channel4->CNDTR = size; // Mettre à jour la taille des données
        DMA1_Channel4->CCR |= DMA_CCR_EN; // Activer le canal DMA
    } else if (usart == 2) {
        memcpy(tx_buffer, data, size); // Copier les données dans le buffer de transmission
        DMA1_Channel7->CNDTR = size; // Mettre à jour la taille des données
        DMA1_Channel7->CCR |= DMA_CCR_EN; // Activer le canal DMA
    } else if (usart == 3) {
        memcpy(tx_buffer, data, size); // Copier les données dans le buffer de transmission
        DMA1_Channel2->CNDTR = size; // Mettre à jour la taille des données
        DMA1_Channel2->CCR |= DMA_CCR_EN; // Activer le canal DMA
    }
}

void USART_DMA_RX(unsigned short usart, uint8_t *data, int size) {

    if (usart == 1) {
        DMA1_Channel5->CMAR = (uint32_t)data; // Pointer le buffer de réception vers l'adresse fournie
        DMA1_Channel5->CNDTR = size; // Mettre à jour la taille des données
        DMA1_Channel5->CCR |= DMA_CCR_EN; // Activer le canal DMA
    } else if (usart == 2) {
        DMA1_Channel6->CMAR = (uint32_t)data; // Pointer le buffer de réception vers l'adresse fournie
        DMA1_Channel6->CNDTR = size; // Mettre à jour la taille des données
        DMA1_Channel6->CCR |= DMA_CCR_EN; // Activer le canal DMA
    } else if (usart == 3) {
        DMA1_Channel3->CMAR = (uint32_t)data; // Pointer le buffer de réception vers l'adresse fournie
        DMA1_Channel3->CNDTR = size; // Mettre à jour la taille des données
        DMA1_Channel3->CCR |= DMA_CCR_EN; // Activer le canal DMA
    }
}

void USART1_IRQHandler(void) {

    if (USART1->SR & USART_SR_RXNE) {
        uint8_t received = USART1->DR; // Lire les données reçues
        // Traiter les données reçues
    }
}

void USART2_IRQHandler(void) {

    if (USART2->SR & USART_SR_RXNE) {
        uint8_t received = USART2->DR; // Lire les données reçues
        // Traiter les données reçues
    }
}

void USART3_IRQHandler(void) {

    if (USART3->SR & USART_SR_RXNE) {
        uint8_t received = USART3->DR; // Lire les données reçues
        // Traiter les données reçues
    }
}

void DMA1_Channel4_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF4) {
        DMA1->IFCR |= DMA_IFCR_CTCIF4; // Effacer le drapeau de fin de transfert
        // Traiter la fin de la transmission
    }
}

void DMA1_Channel5_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF5) {
        DMA1->IFCR |= DMA_IFCR_CTCIF5; // Effacer le drapeau de fin de transfert
        // Traiter la fin de la réception
    }
}

void DMA1_Channel6_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF6) {
        DMA1->IFCR |= DMA_IFCR_CTCIF6; // Effacer le drapeau de fin de transfert
        // Traiter la fin de la réception
    }
}

void DMA1_Channel7_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF7) {
        DMA1->IFCR |= DMA_IFCR_CTCIF7; // Effacer le drapeau de fin de transfert
        // Traiter la fin de la transmission
    }
}

void DMA1_Channel2_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR |= DMA_IFCR_CTCIF2; // Effacer le drapeau de fin de transfert
        // Traiter la fin de la transmission
    }
}

void DMA1_Channel3_IRQHandler(void) {

    if (DMA1->ISR & DMA_ISR_TCIF3) {
        DMA1->IFCR |= DMA_IFCR_CTCIF3; // Effacer le drapeau de fin de transfert
        // Traiter la fin de la réception
    }
}
