/*
 * MEM2067.c
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/MEM2067.h"


extern FATFS USERFatFS;
extern char USERPath[4];

// Dedebugging
const char* FATFS_ErrorToString(FRESULT result) {

    switch (result) {
        case FR_OK: return "Succeeded";
        case FR_DISK_ERR: return "A hard error occurred in the low level disk I/O layer";
        case FR_INT_ERR: return "Assertion failed";
        case FR_NOT_READY: return "The physical drive cannot work";
        case FR_NO_FILE: return "Could not find the file";
        case FR_NO_PATH: return "Could not find the path";
        case FR_INVALID_NAME: return "The path name format is invalid";
        case FR_DENIED: return "Access denied due to prohibited access or directory full";
        case FR_EXIST: return "Access denied due to prohibited access";
        case FR_INVALID_OBJECT: return "The file/directory object is invalid";
        case FR_WRITE_PROTECTED: return "The physical drive is write protected";
        case FR_INVALID_DRIVE: return "The logical drive number is invalid";
        case FR_NOT_ENABLED: return "The volume has no work area";
        case FR_NO_FILESYSTEM: return "There is no valid FAT volume";
        case FR_MKFS_ABORTED: return "The f_mkfs() aborted due to any parameter error";
        case FR_TIMEOUT: return "Could not get a grant to access the volume within defined period";
        case FR_LOCKED: return "The operation is rejected according to the file sharing policy";
        case FR_NOT_ENOUGH_CORE: return "LFN working buffer could not be allocated";
        case FR_TOO_MANY_OPEN_FILES: return "Number of open files > _FS_SHARE";
        case FR_INVALID_PARAMETER: return "Given parameter is invalid";
        default: return "Unknown error";
    }
}

uint8_t MEM2067_WriteFATFS(const char *filename, uint8_t *data, uint16_t size) {

    FIL file;
    FRESULT result;
    UINT bytes_written;

    result = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (result != FR_OK) {
        printf("Error opening file: %s\n", FATFS_ErrorToString(result));
        return 0;
    }

    result = f_write(&file, data, size, &bytes_written);
    if (result != FR_OK || bytes_written != size) {
        printf("Error writing: %s\n", FATFS_ErrorToString(result));
        f_close(&file);
        return 0;
    }

    f_close(&file);
    return 1; // OK
}

uint8_t MEM2067_SDCardDetection(void) {

	memset(&USERFatFS, 0, sizeof(USERFatFS));
	memset(USERPath, 0, sizeof(USERPath));

    FRESULT res = f_mount(&USERFatFS, USERPath, 1);
    if (res == FR_OK) {
        f_mount(NULL, USERPath, 1);
        printf(" -> SD card detected\r\n");
        return 1; // OK
    } else {
        printf(" -> No SD card detected\r\n");
        printf(" -> SD card error, result: %s\n", FATFS_ErrorToString(res));
        return 0;
    }
}
