#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"

void Init_GPIO(GPIO_TypeDef *port, unsigned short pin, unsigned short dir, unsigned short opt) {

    volatile uint32_t *CR;
    unsigned short pinIndex = pin;

    // Activer l'horloge du port GPIO
    if (port == GPIOA) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Activer PORT A
    } else if (port == GPIOB) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Activer PORT B
    } else if (port == GPIOC) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Activer PORT C
    } else return;

    // Choisir le bon registre de configuration (CRL ou CRH)
    CR = (pin < 8) ? &(port->CRL) : &(port->CRH);
    if (pin >= 8) {
    	pinIndex -= 8;
    }

    // Configurer la direction et l'option du PIN
    *CR &= ~(0xF << (pinIndex * 4)); // Réinitialiser le pin cible
    *CR |= (dir << (pinIndex * 4)) | (opt << (pinIndex * 4 + 2)); // Configurer
}

int Read_GPIO(GPIO_TypeDef *port, unsigned short pin) {

	return (port->IDR & (1 << pin)) >> pin;
}

void Write_GPIO(GPIO_TypeDef *port, unsigned short pin, unsigned short state) {

	if (state) {
		port->ODR |= (1 << pin);
	} else port->ODR &= ~(1 << pin);
}

void Toggle_GPIO(GPIO_TypeDef *port, unsigned short pin) {

	if (Read_GPIO(port, pin)) {
		Write_GPIO(port, pin, 0);
	} else Write_GPIO(port, pin, 1);
}
