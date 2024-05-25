/*
 * MEM2067.c
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/MEM2067.h"

uint8_t MEM2067_Init(void) {

    Init_GPIO(PA, 4, OUT10, O_GP_PP);
    SPI_Init(1);
    return 0; // OK
}

uint8_t MEM2067_WriteFATFS(const char *filename, uint8_t *data, uint16_t size) {

	FATFS fs;
    FIL file;
    FRESULT result;
    UINT bytes_written;

    // Montage de la partition FATFS
    if (f_mount(&fs, "", 1) != FR_OK) {
        return 1;
    }
    // Ouverture fichier
    result = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (result != FR_OK) {
        f_mount(NULL, "", 1); // Démontage si erreur
        return 1;
    }
    // Écriture données
    result = f_write(&file, data, size, &bytes_written);
    if (result != FR_OK) {
       return 1;
    }
    // Fermeture fichier
    f_close(&file);

    // Démontage FATFS
    f_mount(NULL, "", 1);

    return 0; // OK
}
