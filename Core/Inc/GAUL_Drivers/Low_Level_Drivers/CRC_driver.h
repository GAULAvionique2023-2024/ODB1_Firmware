/*
 * CRC.h
 *
 *  Created on: Feb 16, 2024
 *      Author: gagno
 */

#ifndef INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_H_BAK_
#define INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_H_BAK_

#define CRC_REG_BASE 0x40023000
#define CRC_REG_OFFS_DR 0x00 // Stocke les données à traiter et le résultat du calcul CRC
#define CRC_REG_OFFS_CR 0x08 // Contrôle le fonctionnement du CRC, y compris la sélection du polynôme générateur, le mode de calcul et le mode d'initialisation

#define CRC_REG_POLY_COMPUTATION 0x4C11DB7
#define CRC_REG_INIT 0xFFFFFFFF // Reset CRC_DR
//4 cycles AHB (HCLK)

typedef struct {

	CRC_HandleTypeDef CRC_Handle;

} CRC32;

// Low level CRC functions
void CRC32_Init(CRC32 *hcrc);
uint32_t CRC32_GetCRC(CRC32 *hcrc, uint32_t *addressData, uint32_t length);

#endif /* INC_GAUL_DRIVERS_LOW_LEVEL_DRIVERS_CRC_H_BAK_ */
