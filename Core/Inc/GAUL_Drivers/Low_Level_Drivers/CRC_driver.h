/*
 * CRC_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_DRIVER_H_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_DRIVER_H_


uint32_t CRC32_Calculate(CRC_HandleTypeDef *hcrc, uint32_t inputBuffer[], uint32_t length); // Retourne le contenu calculé du registre CRC


#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_DRIVER_H_ */
