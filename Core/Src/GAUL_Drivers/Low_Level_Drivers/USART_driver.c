/*
 * UART_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#include "main.h"
#include <GAUL_Drivers/Low_Level_Drivers/USART_driver.h>
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"

void USART_Init(unsigned short usart)
{
	if(usart == 1)
	{
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

		Init_GPIO(PB, 6, OUT50, O_AF_PP); // TX
		Init_GPIO(PB, 7, IN, I_PP); // RX

		USART1->CR1 |= USART_CR1_UE; // Activer USART (0x0C)
		USART1->CR1 |= USART_CR1_TE; // Activer la transmission
		USART1->CR1 |= USART_CR1_RE; // Activer la réception
	}
	else if(usart == 2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

		Init_GPIO(PA, 2, OUT50, O_AF_PP); // TX
		Init_GPIO(PA, 3, IN, I_PP); // RX

		USART2->CR1 |= USART_CR1_UE; // Activer USART (0x10)
		USART2->CR1 |= USART_CR1_TE; // Activer la transmission
		USART2->CR1 |= USART_CR1_RE; // Activer la réception
	}
	else if(usart == 3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN ;

		Init_GPIO(PB, 10, OUT50, O_AF_PP); // TX
		Init_GPIO(PB, 11, IN, I_PP); // RX

		USART3->CR1 |= USART_CR1_UE; // Activer USART (0x14)
		USART3->CR1 |= USART_CR1_TE; // Activer la transmission
		USART3->CR1 |= USART_CR1_RE; // Activer la réception
	}
}

void USART_TX(unsigned short usart, uint8_t *data, int size) {

	if(usart == 1)
	{
		int i = 0;
		while (i < size)
		{
			while (!(USART1->SR & USART_SR_TXE));
			USART1->DR = data[i];
			i++;
		}
		while (!(USART1->SR & USART_SR_TC));
	}
	else if(usart == 2)
	{
		int i = 0;
		while (i < size)
		{
			while (!(USART2->SR & USART_SR_TXE));
			USART2->DR = data[i];
			i++;
		}
		while (!(USART2->SR & USART_SR_TC));
	}
	else if(usart == 3)
	{
		int i = 0;
		while (i < size)
		{
			while (!(USART3->SR & USART_SR_TXE));
			USART3->DR = data[i];
			i++;
		}
		while (!(USART3->SR & USART_SR_TC));
	}
}

void USART_RX(unsigned short usart, uint8_t *data, int size) {

	if(usart == 1)
	{
		int i = 0;
		while (i < size)
		{
			while (!(USART1->SR & USART_SR_RXNE));
			data[i] = USART1->DR;
			i++;
		}
	}
	else if(usart == 2)
	{
		int i = 0;
		while (i < size)
		{
			while (!(USART2->SR & USART_SR_RXNE));
			data[i] = USART2->DR;
			i++;
		}
	}
	else if(usart == 3)
	{
		int i = 0;
		while (i < size)
		{
			while (!(USART3->SR & USART_SR_RXNE));
			data[i] = USART3->DR;
			i++;
		}
	}
}
