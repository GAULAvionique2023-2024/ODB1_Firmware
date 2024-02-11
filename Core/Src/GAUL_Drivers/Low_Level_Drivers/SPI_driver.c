/*
 * SPI_driver.c
 *
 *  Created on: Feb 11, 2024
 *      Author: Luka
 */

#include "main.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"


void SPI_Init(unsigned short spi)
{

	RCC->APB2ENR |= 1;

	if(spi == 1)
	{
		RCC->APB2ENR |= 0x1000;

		Init_GPIO(PA, 4, OUT50, O_GP_PP);
		Init_GPIO(PA, 5, OUT50, O_GP_PP);
		Init_GPIO(PA, 6,    IN,    I_PP);
		Init_GPIO(PA, 7, OUT50, O_GP_PP);

		SPI1->CR1 |= 0x04; //Master mode
		SPI1->CR1 |= 0x10; //flck / 8
		SPI1->CR1 |= 0x40; //Enabling SPI
		SPI1->CR2 |= 0x04;

		Write_GPIO(PA,  4, HIGH);
	}
	else if(spi == 2)
	{
		RCC->APB1ENR |= 0x4000;

		Init_GPIO(PA,  8, OUT50, O_GP_PP);
		Init_GPIO(PB, 12, OUT50, O_GP_PP);
		Init_GPIO(PB, 13, OUT50, O_GP_PP);
		Init_GPIO(PB, 14,    IN,    I_PP);
		Init_GPIO(PB, 15, OUT50, O_GP_PP);

		SPI2->CR1 |= 0x04; //Master mode
		SPI2->CR1 |= 0x10; //flck / 8
		SPI2->CR1 |= 0x40; //Enabling SPI
		SPI2->CR2 |= 0x04;

		Write_GPIO(PA,  8, HIGH);
		Write_GPIO(PB, 12, HIGH);
	}
}

void SPI1_TX_Char(char tx_char)
{
	SPI1->DR = tx_char;
	while(SPI1->SR & 0x80);
}

void SPI2_TX_Char(char tx_char)
{
	SPI2->DR = tx_char;
	while(SPI2->SR & 0x80);
}

void SPI1_TX_String(char str[])
{
	int i = 0;

	while(str[i])
	{
		SPI1->DR = str[i];
		while(SPI1->SR & 0x80);
		i++;
	}
}

void SPI2_TX_String(char str[])
{
	int i = 0;

	while(str[i])
	{
		SPI2->DR = str[i];
		while(SPI2->SR & 0x80);
		i++;
	}
}

char SPI1_RX(char data)
{
	char rx_val = 0xFF;

	SPI1->DR = data;while(SPI1->SR & 0x80);
	while(SPI1->SR & 0x01){rx_val = SPI1->DR;}

	return rx_val;
}

char SPI2_RX(char data)
{
	char rx_val = 0xFF;

	SPI2->DR = data;while(SPI2->SR & 0x80);
	while(SPI2->SR & 0x01){rx_val = SPI2->DR;}

	return rx_val;
}
