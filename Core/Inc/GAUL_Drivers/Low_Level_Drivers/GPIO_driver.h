#ifndef INC_GAUL_DRIVERS_GP_DRIVE_H_
#define INC_GAUL_DRIVERS_GP_DRIVE_H_

#define RCC_APB2ENR     (*((volatile unsigned long *) 0x40021018))

#define GPIO_A_BASE     0x40010800
#define GPIO_B_BASE     0x40010C00
#define GPIO_C_BASE     0x40011000

#define GPIO_A          ((volatile unsigned long *) GPIO_A_BASE)
#define GPIO_B          ((volatile unsigned long *) GPIO_B_BASE)
#define GPIO_C          ((volatile unsigned long *) GPIO_C_BASE)

// Register offsets
#define GPIO_CRL        0x00
#define GPIO_CRH        0x04
#define GPIO_IDR        0x08
#define GPIO_ODR        0x0C

// List of Ports
#define PA 1   // Port A
#define PB 2   // Port B
#define PC 3   // Port C

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

void Init_GPIO(unsigned short port, unsigned short pin, unsigned short dir, unsigned short opt);

int Read_GPIO(unsigned short port, unsigned short pin);

void Write_GPIO(unsigned short port, unsigned short pin, unsigned short state);

void Toggle_GPIO(unsigned short port, unsigned short pin);

#endif /* INC_GAUL_DRIVERS_GP_DRIVE_H_ */
