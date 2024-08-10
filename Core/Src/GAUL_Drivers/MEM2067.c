/*
 * MEM2067.c
 *
 *  Created on: May 24, 2024
 *      Author: gagno
 */

#include "GAUL_Drivers/MEM2067.h"

static FATFS fs;
static FIL fil;
static FRESULT fresult;

static FATFS *pfr;
static DWORD fre_clust;

void MEM2067_Write(const char *filename, const char* data);
void MEM2067_Unmount(void);

uint8_t MEM2067_Mount(const char* filename) {

	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK){
		return 0;
	}
	// Create file with read / write access and open it
	MEM2067_Write(filename, "LOG\n");

	return 1;
}

void MEM2067_Write(const char *filename, const char* data) {

	fresult = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
	f_lseek(&fil, f_size(&fil));
	f_puts(data, &fil);

	f_close(&fil);
}

char *MEM2067_Read(const char *filename) {

	char *data = "";

	fresult = f_open(&fil, filename, FA_OPEN_ALWAYS | FA_WRITE);
	f_gets(data, sizeof(data), &fil);

	f_close(&fil);

	return data;
}

void MEM2067_Unmount(void) {

	fresult = f_mount(NULL, "/", 1);
}

void MEM2067_Infos(MEM2067 *devMEM) {

	f_getfree("", &fre_clust, &pfr);
	devMEM->total_space = (uint32_t)((pfr->n_fatent - 2) * pfr->csize * 0.5);
	devMEM->free_space = (uint32_t)(fre_clust * pfr->csize * 0.5);
}

int bufsize (char* buf)
{
	int i = 0;
	while (*buf++ != '\0')
		i++;
	return i;
}

void bufclear(char* p_Buffer) {

	for (int i = 0; i < BUFFER_SIZE; i++) {
		p_Buffer[i] = '\0';
	}
}

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
        default: return "Unknown error\r\n";
    }
}
