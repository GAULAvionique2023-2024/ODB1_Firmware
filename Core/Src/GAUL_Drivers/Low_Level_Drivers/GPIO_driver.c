#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"

void Enable_GPIO_Clock(GPIO_TypeDef *port) {

    if (port == GPIOA) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; // Activer PORT A
    } else if (port == GPIOB) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPBEN; // Activer PORT B
    } else if (port == GPIOC) {
        RCC->APB2ENR |= RCC_APB2ENR_IOPCEN; // Activer PORT C
    }
}

// (CRL / CRH)
volatile uint32_t* Select_CR(GPIO_TypeDef *port, unsigned short pin) {

    return (pin < 8) ? &(port->CRL) : &(port->CRH);
}

void Init_GPIO(GPIO_TypeDef *port, unsigned short pin, unsigned short dir, unsigned short opt) {

    Enable_GPIO_Clock(port);

    volatile uint32_t *CR = Select_CR(port, pin);
    unsigned short pinIndex = (pin < 8) ? pin : pin - 8;

	// Configurer la direction et l'option du PIN
	*CR &= ~(0xF << (pinIndex * 4)); // Réinitialiser le pin cible
	*CR |= (dir << (pinIndex * 4)) | (opt << (pinIndex * 4 + 2));
}

// Only PA9
void Init_Interrupt_GPIO(GPIO_TypeDef *port, unsigned short pin) {

	Init_GPIO(port, pin, IN, I_PP);

	// Configurer l'interruption externe (EXTI) sur PA9
	LL_GPIO_AF_SetEXTISource(LL_GPIO_AF_EXTI_PORTA, LL_GPIO_AF_EXTI_LINE9);
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_9);
	LL_EXTI_EnableRisingTrig_0_31(LL_EXTI_LINE_9);

	// Configurer la priorité de l'interruption et l'activer
	NVIC_SetPriority(EXTI9_5_IRQn, 0);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}

int Read_GPIO(GPIO_TypeDef *port, unsigned short pin) {

    return (port->IDR & (1 << pin)) >> pin;
}

void Write_GPIO(GPIO_TypeDef *port, unsigned short pin, unsigned short state) {

    if (state) {
        port->ODR |= (1 << pin);
    } else {
        port->ODR &= ~(1 << pin);
    }
}

void Toggle_GPIO(GPIO_TypeDef *port, unsigned short pin) {

    port->ODR ^= (1 << pin);
}
