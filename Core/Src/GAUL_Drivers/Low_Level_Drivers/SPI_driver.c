/*
 * SPI_driver.c
 *
 *  Created on: Feb 11, 2024
 *      Author: Luka
 */

#include "main.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"

void SPI_Init(unsigned short spi) {
    if(spi == 1) {
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        Init_GPIO(PA, 4, OUT50, O_GP_PP); // CS
        Init_GPIO(PA, 5, OUT50, O_AF_PP); // CLK
        Init_GPIO(PA, 6, IN, I_PP);    // MISO SPI1
        Init_GPIO(PA, 7, OUT50, O_AF_PP); // MOSI SPI1

        Write_GPIO(PA, 4, HIGH);

        SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    } else if(spi == 2) {
        RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

        Init_GPIO(PA, 8, OUT50, O_GP_PP); // CS (optionnel)
        Init_GPIO(PB, 12, OUT50, O_GP_PP); // CS
        Init_GPIO(PB, 13, OUT50, O_AF_PP); // CLK
        Init_GPIO(PB, 14, IN, I_PP);    // MISO SPI2
        Init_GPIO(PB, 15, OUT50, O_AF_PP); // MOSI SPI2

        Write_GPIO(PA, 8, HIGH);
        Write_GPIO(PB, 12, HIGH);

        SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_BR_2 | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_SPE;
    }
}

void SPI1_TX(uint8_t *data, int size) {

    for (int i = 0; i < size; ++i) {
        while (!(SPI1->SR & SPI_SR_TXE)) {}
        SPI1->DR = data[i];
    }

    while (!(SPI1->SR & SPI_SR_TXE)) {}
    while (SPI1->SR & SPI_SR_BSY) {}

    uint8_t temp = SPI1->DR;
    temp = SPI1->SR;
}

void SPI2_TX(uint8_t *data, int size) {

    for (int i = 0; i < size; ++i) {
        while (!(SPI2->SR & SPI_SR_TXE)) {}
        SPI2->DR = data[i];
    }

    while (!(SPI2->SR & SPI_SR_TXE)) {}
    while (SPI2->SR & SPI_SR_BSY) {}

    uint8_t temp = SPI2->DR;
    temp = SPI2->SR;
}

void SPI1_RX(uint8_t *data, int size) {

    while (size) {
        while (SPI1->SR & SPI_SR_BSY) {}
        SPI1->DR = 0;
        while (!(SPI1->SR & SPI_SR_RXNE)) {}
        *data++ = SPI1->DR;
        --size;
    }
}

void SPI2_RX(uint8_t *data, int size) {

    while (size) {
        while (SPI2->SR & SPI_SR_BSY) {}
        SPI2->DR = 0;
        while (!(SPI2->SR & SPI_SR_RXNE)) {}
        *data++ = SPI2->DR;
        --size;
    }
}
