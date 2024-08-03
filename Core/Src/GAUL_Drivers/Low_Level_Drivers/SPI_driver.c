#include "main.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"

#define TIMEOUT 200  // Timeout value

void SPI_Init(SPI_TypeDef *SPIx) {
    if (SPIx == SPI1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        Init_GPIO(PA, 4, OUT50, O_GP_PP); // CS
        Init_GPIO(PA, 5, OUT50, O_AF_PP); // CLK
        Init_GPIO(PA, 6, IN, I_PP);    // MISO SPI1
        Init_GPIO(PA, 7, OUT50, O_AF_PP); // MOSI SPI1

        Write_GPIO(PA, 4, HIGH);

        SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    } else if (SPIx == SPI2) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

        Init_GPIO(PB, 13, OUT50, O_AF_PP); // CLK
        Init_GPIO(PB, 14, IN, I_PP);    // MISO SPI2
        Init_GPIO(PB, 15, OUT50, O_AF_PP); // MOSI SPI2

        SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    }
}

int SPI_TX(SPI_TypeDef *SPIx, uint8_t *data, int size) {
    uint32_t timeout = TIMEOUT;
    while (size--) {
        // Attendre que le buffer TX soit vide
        while (!(SPIx->SR & SPI_SR_TXE)) {
            if (--timeout == 0) {
                return -1; // Timeout
            }
        }
        SPIx->DR = *data++; // Envoyer les données
    }

    // Attendre que la transmission soit terminée
    timeout = TIMEOUT;
    while (SPIx->SR & SPI_SR_BSY) {
        if (--timeout == 0) {
            return -1; // Timeout
        }
    }

    // Lire le registre pour vider le buffer RX
    (void)SPIx->DR;
    return 0; // Succès
}

int SPI_RX(SPI_TypeDef *SPIx, uint8_t *data, int size) {
    uint32_t timeout = TIMEOUT;
    while (size--) {
        // Envoyer un dummy byte pour générer un clock et recevoir des données
        while (!(SPIx->SR & SPI_SR_TXE)) {
            if (--timeout == 0) {
                return -1; // Timeout
            }
        }
        SPIx->DR = 0xFF; // Dummy byte

        // Attendre que le buffer RX contienne des données
        timeout = TIMEOUT;
        while (!(SPIx->SR & SPI_SR_RXNE)) {
            if (--timeout == 0) {
                return -1; // Timeout
            }
        }
        *data++ = SPIx->DR; // Lire les données
    }
    return 0; // Succès
}
