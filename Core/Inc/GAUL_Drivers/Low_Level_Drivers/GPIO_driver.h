/*
 * gp_drive.h
 *
 *  Created on: Feb 11, 2024
 *      Author: Luka
 */

#ifndef INC_GAUL_DRIVERS_GP_DRIVE_H_
#define INC_GAUL_DRIVERS_GP_DRIVE_H_


#define RCC_APB2ENR     (*((volatile unsigned long *) 0x40021018))

#define GPIO_A          (*((volatile unsigned long *) 0x40010800))
#define GPIO_B          (*((volatile unsigned long *) 0x40010C00))
#define GPIO_C          (*((volatile unsigned long *) 0x40011000))

/// List of Ports
#define PA 1		// Port A
#define PB 2		// Port B
#define PC 3		// Port C

/// Directions
#define IN 0 		// Indique que la broche gpio est configurée comme entrée
#define OUT10 1 	// "" comme sortie (10MHz)
#define OUT2  2		// "" comme sortie (2MHz)
#define OUT50 3		// "" comme sortie (50MHz)

/// Direction options for input
#define I_AN 0		// Indique que la broche gpio est configurée comme entrée analogique
#define I_F  1		// "" comme entrée numérique avec détection des deux fronts (montant et descendant)
#define I_PP 2		// "" comme entrée numérique avec une configuration push-pull

/// Direction options for output
#define O_GP_PP 0	// Indique que la broche gpio est configurée comme sortie avec une configuration push-pull
#define O_GP_OD 1	// "" comme sortie à drain ouvert
#define O_AF_PP 2	// "" comme sortie push-pull en mode alternatif
#define O_AF_OD 3	// "" comme sortie à drain ouvert en mode alternatif

/// State definition
#define LOW  0
#define HIGH 1


void Init_GPIO(unsigned short port, unsigned short pin, unsigned short dir, unsigned short opt);

int Read_GPIO(unsigned short port, unsigned short pin);

void Write_GPIO(unsigned short port, unsigned short pin, unsigned short state);

void Toggle_GPIO(unsigned short port, unsigned short pin);

#endif /* INC_GAUL_DRIVERS_GP_DRIVE_H_ */
