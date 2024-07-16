/*
 * MEM2067.c
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/MEM2067.h"

// Debugging
const char* FATFS_ErrorToString(FRESULT result) {

    switch (result) {
        case FR_OK: return "Succeeded\r\n";
        case FR_DISK_ERR: return "A hard error occurred in the low level disk I/O layer\r\n";
        case FR_INT_ERR: return "Assertion failed\r\n";
        case FR_NOT_READY: return "The physical drive cannot work\r\n";
        case FR_NO_FILE: return "Could not find the file\r\n";
        case FR_NO_PATH: return "Could not find the path\r\n";
        case FR_INVALID_NAME: return "The path name format is invalid\r\n";
        case FR_DENIED: return "Access denied due to prohibited access or directory full\r\n";
        case FR_EXIST: return "Access denied due to prohibited access\r\n";
        case FR_INVALID_OBJECT: return "The file/directory object is invalid\r\n";
        case FR_WRITE_PROTECTED: return "The physical drive is write protected\r\n";
        case FR_INVALID_DRIVE: return "The logical drive number is invalid\r\n";
        case FR_NOT_ENABLED: return "The volume has no work area\r\n";
        case FR_NO_FILESYSTEM: return "There is no valid FAT volume\r\n";
        case FR_MKFS_ABORTED: return "The f_mkfs() aborted due to any parameter error\r\n";
        case FR_TIMEOUT: return "Could not get a grant to access the volume within defined period\r\n";
        case FR_LOCKED: return "The operation is rejected according to the file sharing policy\r\n";
        case FR_NOT_ENOUGH_CORE: return "LFN working buffer could not be allocated\r\n";
        case FR_TOO_MANY_OPEN_FILES: return "Number of open files > _FS_SHARE\r\n";
        case FR_INVALID_PARAMETER: return "Given parameter is invalid\r\n";
        default: return "Unknown error";
    }
}

uint8_t MEM2067_Mount(void) {

	FATFS fatfs;
	FIL fil;
	FRESULT fr_result;
	FATFS *fr_ptr;
	UINT rwc, wwc;
	DWORD free_clusters;
	uint32_t total_size, free_space;
	char rw_buffer[200];

	//------------------[ Mount The SD Card ]--------------------
	Init_GPIO(PA, 4, OUT2, O_GP_PP);
	SPI_Init(1);
	Write_GPIO(PA, 4, HIGH);
	fr_result = f_mount(&fatfs, "", 1);
	if(fr_result != FR_OK) {
		printf("Failed: %s", FATFS_ErrorToString(fr_result));
		return 0;
	}
	printf("Succeeded: %s", FATFS_ErrorToString(fr_result));
	//------------------[ Get & Print The SD Card Size & Free Space ]--------------------
	f_getfree("", &free_clusters, &fr_ptr);
	total_size = (uint32_t)((fr_ptr->n_fatent - 2) * fr_ptr->csize * 0.5);
	free_space = (uint32_t)(free_clusters * fr_ptr->csize * 0.5);
	printf("Total SD Card Size: %lu Bytes\r\n", total_size);
	printf("Free SD Card Space: %lu Bytes\r\n\n", free_space);

	return 1;
}
/*
static void MEM2067_Read(const char *filename, ) {

	//------------------[ Open A Text File For Write & Write Data ]--------------------
	//Open the file
	fr_result = f_open(&fil, "log.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
	printf("Result: %s", FATFS_ErrorToString(fr_result));
}
*/
