/*
 * hcrc.c
 *
 *  Created on: Feb 16, 2024
 *      Author: gagno
 */


// Utiliser plutôt les registre define CRC_driver.h
void CRC32_Init(CRC32 *hcrc, CRC_HandleTypeDef CRC_Handle) {

	hcrc->CRC_Handle = CRC_Handle;

	CRC_Handle.Instance = CRC;

	CRC_Handle.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE; // CRC_REG_POLY_COMPUTATION 0x4C11DB7
	CRC_Handle.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE; //
	CRC_Handle.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
	CRC_Handle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
	CRC_Handle.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
}

uint32_t CRC32_GetCRC(CRC32 *hcrc, uint32_t *addressData, uint32_t length) {

	return HAL_CRC_Calculate(hcrc, addressData, length);
}




// Exemple main.c :
//uint8_t data[] = "Ceci est un exemple de texte";
//uint32_t size = strlen(data);

//hcrc32 hcrc;
//uint32_t hcrc_value;

//hcrc32_Init(&hcrc);

//hcrc_value = hcrc32_Calculation(&hcrc, data, size);

//printf("Le hcrc est : 0x%08X\n", hcrc_value);




