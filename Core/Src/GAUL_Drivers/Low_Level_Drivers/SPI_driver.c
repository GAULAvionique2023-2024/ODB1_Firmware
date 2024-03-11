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

		Init_GPIO(PA, 4, OUT50, O_GP_PP); //CS
		Init_GPIO(PA, 5, OUT50, O_AF_PP); //CLK
		Init_GPIO(PA, 6,    IN,    I_PP); //MISO SPI1
		Init_GPIO(PA, 7, OUT50, O_AF_PP); //MOSI SPI1

		Write_GPIO(PA,  4, HIGH);

		SPI1->CR1 |= 0x04; 	//Master mode
		SPI1->CR1 |= 0x28; 	//flck / 64
		SPI1->CR1 |= 0x300;	// SSM=1, SSI=1
		SPI1->CR1 |= 0x40; 	//Enabling SPI
	}
	else if(spi == 2)
	{
		RCC->APB1ENR |= 0x4000;

		Init_GPIO(PA,  8, OUT50, O_GP_PP); //CS
		Init_GPIO(PB, 12, OUT50, O_GP_PP); //CS
		Init_GPIO(PB, 13, OUT50, O_AF_PP); //CLK
		Init_GPIO(PB, 14,    IN,    I_PP); //MISO SPI2
		Init_GPIO(PB, 15, OUT50, O_AF_PP); //MOSI SPI2

		Write_GPIO(PA,  8, HIGH);
		Write_GPIO(PB, 12, HIGH);

		SPI2->CR1 |= 0x04; 	//Master mode
		SPI2->CR1 |= 0x28; 	//flck / 64
		SPI2->CR1 |= 0x300; // SSM=1, SSI=1
		SPI2->CR1 |= 0x40; 	//Enabling SPI
	}
}


void SPI1_TX(uint8_t *data, int size)
{
	int i=0;
		while (i<size)
		{
			while (!((SPI1->SR)& 0x02)) {};
		   SPI1->DR = data[i];  // load the data into the Data Register
		   i++;
		}

		while (!((SPI1->SR)&(1<<1))) {};
		while (((SPI1->SR)&(1<<7))) {};

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPI1->DR;
	temp = SPI1->SR;
}

void SPI2_TX(uint8_t *data, int size)
{
	int i=0;
		while (i<size)
		{
			while (!((SPI2->SR)& 0x02)) {};
		   SPI2->DR = data[i];  // load the data into the Data Register
		   i++;
		}

		while (!((SPI2->SR)&(1<<1))) {};
		while (((SPI2->SR)&(1<<7))) {};

	//  Clear the Overrun flag by reading DR and SR
	uint8_t temp = SPI2->DR;
	temp = SPI2->SR;
}

void SPI1_RX(uint8_t *data, int size)
{
	while (size)
	{
		while (((SPI1->SR)&(1<<7))){};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI1->DR = 0;  // send dummy data
		while (!((SPI1->SR)&(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPI1->DR);
		size--;
	}
}

void SPI2_RX(uint8_t *data, int size)
{
	while (size)
	{
		while (((SPI2->SR)&(1<<7))){};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication
		SPI2->DR = 0;  // send dummy data
		while (!((SPI2->SR)&(1<<0))){};  // Wait for RXNE to set -> This will indicate that the Rx buffer is not empty
	  *data++ = (SPI2->DR);
		size--;
	}
}
