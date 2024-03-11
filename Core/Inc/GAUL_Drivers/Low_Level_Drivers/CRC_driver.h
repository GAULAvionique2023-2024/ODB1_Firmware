/*
 * CRC_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_DRIVER_H_

#define POLYNOMIAL_COMPUTATION 0x1021


uint16_t CRC16_Calculate(uint8_t *data, uint16_t length); // Retourne le contenu calculé du registre CRC


#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_DRIVER_H_ */
