#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"

void Init_GPIO(unsigned short port, unsigned short pin, unsigned short dir, unsigned short opt) {
    volatile unsigned long *CR;
    unsigned short tPIN = pin;
    unsigned short offset = (pin < 8) ? GPIO_CRL : GPIO_CRH;

    if (pin >= 8) tPIN -= 8;

    if (port == 1) {
        RCC_APB2ENR |= (1 << 2); // Activer PORT A
        CR = (volatile unsigned long *)((unsigned long)GPIO_A + offset);
    } else if (port == 2) {
        RCC_APB2ENR |= (1 << 3); // Activer PORT B
        CR = (volatile unsigned long *)((unsigned long)GPIO_B + offset);
    } else if (port == 3) {
        RCC_APB2ENR |= (1 << 4); // Activer PORT C
        CR = (volatile unsigned long *)((unsigned long)GPIO_C + offset);
    }

    *CR &= ~(0xF << (tPIN * 4)); // Réinitialiser le pin cible
    *CR |= (dir << (tPIN * 4)) | (opt << (tPIN * 4 + 2)); // Configurer la direction et l'option du PIN
}

int Read_GPIO(unsigned short port, unsigned short pin) {
    volatile unsigned long *IDR;
    unsigned long offset = GPIO_IDR;

    if (port == 1) {
        IDR = (volatile unsigned long *)((unsigned long)GPIO_A + offset);
    } else if (port == 2) {
        IDR = (volatile unsigned long *)((unsigned long)GPIO_B + offset);
    } else if (port == 3) {
        IDR = (volatile unsigned long *)((unsigned long)GPIO_C + offset);
    }

    return ((*IDR & (1 << pin)) >> pin);
}

void Write_GPIO(unsigned short port, unsigned short pin, unsigned short state) {
    volatile unsigned long *ODR;
    unsigned long offset = GPIO_ODR;

    if (port == 1) {
        ODR = (volatile unsigned long *)((unsigned long)GPIO_A + offset);
    } else if (port == 2) {
        ODR = (volatile unsigned long *)((unsigned long)GPIO_B + offset);
    } else if (port == 3) {
        ODR = (volatile unsigned long *)((unsigned long)GPIO_C + offset);
    }

    if (state) {
        *ODR |= (1 << pin);
    } else {
        *ODR &= ~(1 << pin);
    }
}

void Toggle_GPIO(unsigned short port, unsigned short pin) {
    if (Read_GPIO(port, pin)) {
        Write_GPIO(port, pin, 0);
    } else {
        Write_GPIO(port, pin, 1);
    }
}
