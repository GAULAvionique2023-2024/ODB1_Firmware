/*
 * MEM2067.c
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/MEM2067.h"

uint8_t MEM2067_WriteFATFS(const char *filename, uint8_t *data, uint16_t size) {

	Write_GPIO(PA, 4, LOW);
	printf("SD Card Selected.\n");

	FATFS fs;
	FIL file;
	FRESULT result;
	UINT bytes_written;

	// Montage de la partition FATFS
	result = f_mount(&fs, "", 1);
	if (result != FR_OK) {
		printf("Error mounting: %d\n", result);
		return 1;
	}
	printf("Filesystem mounted.\n");

	// Ouverture fichier
	result = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
	if (result != FR_OK) {
		printf("Error opening file: %d\n", result);
		f_mount(NULL, "", 1); // Démontage si erreur
		return 1;
	}
	printf("File opened: %s\n", filename);

	// Écriture données
	result = f_write(&file, data, size, &bytes_written);
	if (result != FR_OK) {
		printf("Error writing: %d\n", result);
		f_close(&file);
		f_mount(NULL, "", 1); // Démontage si erreur
		return 1;
	}
	printf("Data written: %u bytes\n", bytes_written);

	// Fermeture fichier
	f_close(&file);
	printf("File closed.\n");

	// Démontage FATFS
	f_mount(NULL, "", 1);
	printf("Unmounted.\n");

	return 0; // OK
}

uint8_t MEM2067_SDCardDetection(void) {

    FRESULT res = f_mount(&USERFatFS, (TCHAR const*)USERPath, 1);

    if (res == FR_OK) {
        f_mount(NULL, (TCHAR const*)"", 1);
        return 0; // OK
    } else {
        return 1; // Erreur
    }
}
