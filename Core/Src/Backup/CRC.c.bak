/*
 * CRC.c
 *
 *  Created on: Feb 16, 2024
 *      Author: gagno
 */

void CRC32_Init(CRC32 *crc) {

    crc->polynomial = CRC_REG_POLY_COMPUTATION;
    crc->initialValue = CRC_REG_INIT;
    crc->crc = crc->initialValue;

    // Réinitialisation du registre de contrôle CRC (CR)
    *((uint32_t *)(CRC_REG_BASE + CRC_REG_OFFS_CR)) = 0;

    // Configuration du polynôme CRC
    *((uint32_t *)(CRC_REG_BASE + CRC_REG_OFFS_CR)) |= (crc->polynomial << 4);

    // Activation du calcul CRC
    *((uint32_t *)(CRC_REG_OFFS_CR + CRC_REG_OFFS_CR)) |= 1;
}

// Fonction de réinitialisation
void CRC32_Reset(CRC32 *crc) {

    crc->crc = crc->initialValue;
}

// Fonction d'ajout de données au calcul CRC
void CRC32_AddData(CRC32 *crc, uint8_t data) {
    // Ecriture de la donnée dans le registre de données CRC
    *((uint8_t *)(CRC_REG_OFFS_CR + CRC_REG_OFFS_DR)) = data;
}

// Fonction de récupération du CRC calculé
uint32_t CRC32_GetCRC(CRC32 *crc, uint8_t *data, uint32_t size) {

	CRC32_Calculation(crc->crc, data, sizeof(data));
    return crc->crc;
}

uint32_t CRC32_Calculation(CRC32 *crc, uint8_t *data, uint32_t size) {


	uint32_t i; // Selection de l'octet

	// Réinitialisation du CRC
	CRC32_Reset(crc);
	crc->status = CRC_STATUS_OK;

	// Calcul du CRC pour chaque octet de données
	for (i = 0; i < size; i++)
	{
		CRC32_AddData(crc, data[i]);
	}

	if(crc->crc != CRC_REG_INIT)
	{
		crc->status = CRC_STATUS_ERROR;
	}

	// Retourne le CRC calculé
	return crc->crc;
}

// Exemple main.c :
//uint8_t data[] = "Ceci est un exemple de texte";
//uint32_t size = strlen(data);

//CRC32 crc;
//uint32_t crc_value;

//CRC32_Init(&crc);

//crc_value = CRC32_Calculation(&crc, data, size);

//printf("Le CRC est : 0x%08X\n", crc_value);




