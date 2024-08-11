#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_GPIO_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_GPIO_DRIVER_H_

#include "stm32f1xx_hal.h"

#define RCC_APB2ENR     (*((volatile unsigned long *) 0x40021018))

// Directions
#define IN 0       // Input
#define OUT10 1    // Output 10MHz
#define OUT2  2    // Output 2MHz
#define OUT50 3    // Output 50MHz

// Direction options for input
#define I_AN 0    // Analog input
#define I_F  1    // Input with both edges detection
#define I_PP 2    // Push-pull input

// Direction options for output
#define O_GP_PP 0  // Push-pull output
#define O_GP_OD 1  // Open-drain output
#define O_AF_PP 2  // Alternate function push-pull
#define O_AF_OD 3  // Alternate function open-drain

// State definition
#define LOW  0
#define HIGH 1

void Init_GPIO(GPIO_TypeDef *port, unsigned short pin, unsigned short dir, unsigned short opt);
int Read_GPIO(GPIO_TypeDef *port, unsigned short pin);
void Write_GPIO(GPIO_TypeDef *port, unsigned short pin, unsigned short state);
void Toggle_GPIO(GPIO_TypeDef *port, unsigned short pin);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_GPIO_DRIVER_H_ */
