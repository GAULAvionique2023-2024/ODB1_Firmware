/*
 * util.c
 *
 *  Created on: Aug 4, 2024
 *      Author: Luka
 */

#include "util.h"

// Handlers
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;

// Structs
extern RunTimer run_timer;
extern ROCKET_Data rocket_data;
extern BMP280 bmp_data;
extern ICM20602 icm_data;
extern L76LM33 L76_data;
extern RFD900 rfd_data;

// Buffers
float BMP280_buffer[BMP280_BUFFERSIZE]; // altimeter bmp
// TODO: add hm10 response buffer

// Parameters
int bufferIndex = 0;
int descentCount = 0;
int descentThreshold = 5;
static const float accZMin = 1.1;		// delta > 0.9 g
static const float accResMin = 2.0;		// delta > 2.0 g
static const float angleMin = 5;		// delta > 5 deg

extern char* filename_log;

//extern bool isButton_push = false;

void ROCKET_InitRoutine(void) {

	printt("|----------Starting----------|\r\n");
	RunTimerInit(&run_timer);
	//Buzz(TIM3, LL_TIM_CHANNEL_CH4, START);
	SPI_Init(SPI1);
	SPI_Init(SPI2);
	USART_Init(USART1);
	USART_Init(USART2);
	USART_Init(USART3);
	printt("|----------Components initialization----------|\r\n");
	ROCKET_SetMode(MODE_PREFLIGHT);
	printt("(+) Mode flight: %i succeeded...\r\n", rocket_data.header_states.mode);
	// LED RGB
	WS2812_Init();
	printt("(+) WS2812 succeeded...\r\n");
	// Multiplexer
	if (CD74HC4051_Init(&hadc1) != 1) {
	  printt("(-) CD74HC4051 failed...\r\n");
	} else {
		rocket_data.header_states.pyro0 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_0, VREFPYRO);
		rocket_data.header_states.pyro1 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_1, VREFPYRO);
		printt(" -> Pyro0 state: %i\r\n", rocket_data.header_states.pyro0);
		printt(" -> Pyro1 state: %i\r\n", rocket_data.header_states.pyro1);
		printt("(+) CD74HC4051 succeeded...\r\n");
	}
	// Barometer
	bmp_data.SPIx = SPI2;
	bmp_data.cs_pin = 8;
	bmp_data.cs_port = PA;
	rocket_data.header_states.barometer = BMP280_Init(&bmp_data) == 0 ? 0x01 : 0x00;
	printt(rocket_data.header_states.barometer ? "(+) BMP280 succeeded...\r\n" : "(-) BMP280 failed...\r\n");
	// Accelerometer
	icm_data.SPIx = SPI2;
	icm_data.cs_pin = 12;
	icm_data.cs_port = PB;
	icm_data.int_pin = 10;
	icm_data.int_port = PA;
	rocket_data.header_states.accelerometer = ICM20602_Init(&icm_data) == 0 ? 0x01 : 0x00;
	printt(rocket_data.header_states.accelerometer ? "(+) ICM20602 succeeded...\r\n" : "(-) ICM20602 failed...\r\n");
	// GPS
	rocket_data.header_states.gps = L76LM33_Init(&L76_data, &huart2) == L76LM33_OK ? 0x01 : 0x00;
	printt(rocket_data.header_states.gps ? "(+) L76LM33 succeeded...\r\n" : "(-) L76LM33 failed...\r\n");
	// Radio
	rfd_data.USARTx = USART1;
	rocket_data.header_states.rfd = RFD900_Init(&rfd_data) == 1 ? 0x01 : 0x00;
	printt(rocket_data.header_states.rfd ? "(+) RFD900 succeeded...\r\n" : "(-) RFD900 failed...\r\n");
	// SD Card
	rocket_data.header_states.sd = MEM2067_Mount(filename_log) == 1 ? 0x01 : 0x00;
	printt(rocket_data.header_states.sd ? "(+) SD card succeeded...\r\n" : "(-) SD card failed...\r\n");
	MEM2067_Infos();
	/*
	// Bluetooth
	ble_data.USARTx = USART3;
	HM10BLE_Init(&ble_data);
	*/
	char time[20];

	itoa(L76_data.gps_data.time_raw, time, 10);
	MEM2067_Write(filename_log, time);
}

uint8_t ROCKET_Behavior(void) {

    ICM20602_Update_All(&icm_data);
    BMP280_Read_Temperature_Pressure(&bmp_data);

    uint8_t behavior = 0x00;
    /*
     accZ -> 2bits (orientation / movement)
     rollUp -> 2bits (east / west)
     pitchUp -> 2bits (south / north)
     altitude -> 1bit (check pyro)
     mach lock -> 1bit (check pyro)
     */

    // Orientation Z
    if (icm_data.accZ > 0) {
        behavior |= (1 << 0);	// up
        printt("East: %0.1f\n", icm_data.angleX);
    } else {
        behavior &= ~(1 << 0);	// down
    }
    // Movement
    if (icm_data.accZ <= accZMin && icm_data.accZ >= -accZMin) {
        behavior |= (1 << 1);	// idle z
    } else {
        behavior &= ~(1 << 1);	// move z
    }
    // East
    if (icm_data.angleX >= angleMin) {
        behavior |= (1 << 2); // Detected
        printt("East: %0.1f\n", icm_data.angleX);
    } else {
        behavior &= ~(1 << 2); // Not detected
    }
    // West
    if (icm_data.angleX <= -angleMin) {
        behavior |= (1 << 3); // Detected
        printt("West: %0.1f\n", icm_data.angleX);
    } else {
        behavior &= ~(1 << 3); // Not detected
    }
    // South
    if (icm_data.angleY <= -angleMin) {
        behavior |= (1 << 4);
        printt("South: %0.1f\n", icm_data.angleY);
    } else {
        behavior &= ~(1 << 4);
    }
    // North
    if (icm_data.angleY >= angleMin) {
        behavior |= (1 << 5);
        printt("North: %0.1f\n", icm_data.angleY);
    } else {
        behavior &= ~(1 << 5);
    }
    // Altitude (falling)
    if (Altitude_Trend(bmp_data.altitude_filtered_m) == true) {
        behavior |= (1 << 6);
        printt("Rocket is descending\n");
    } else {
        behavior &= ~(1 << 6);
    }
    // Mach Lock (vector norm acceleration)
    if (icm_data.accResult >= accResMin) {
        behavior |= (1 << 7);
        printt("Mach lock: Active\n");
    } else {
        behavior &= ~(1 << 7);
        printt("Mach lock: Disable\n");
    }

    return behavior;
}

uint8_t ROCKET_ModeRoutine(void) {

    uint8_t check = 0;

    uint8_t header_states = 0x00;
    rocket_data.size = 0;
    rocket_data.crc16[0] = 0x00;
    rocket_data.crc16[1] = 0x00;

    /*
     * TODO: add debug mode button interrupt
    if(Read_GPIO(PA, 9) == 1) {
		ROCKET_SetMode(MODE_DEBUG);
		printt("(+) Debug Mode: %i succeeded...\r\n", rocket_data.header_states.mode);
    }
	*/
    switch (rocket_data.header_states.mode) {
    case MODE_PREFLIGHT:
        //BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);

        header_states = (rocket_data.header_states.mode << 6) | (rocket_data.header_states.pyro0 << 5) | (rocket_data.header_states.pyro1 << 4)
                | (rocket_data.header_states.accelerometer << 3) | (rocket_data.header_states.barometer << 2) | (rocket_data.header_states.gps << 1)
                | rocket_data.header_states.sd;

        rocket_data.size = PREFLIGHT_DATASIZE;
        rocket_data.data = (uint8_t*)malloc(rocket_data.size * sizeof(uint8_t));
        // Altitude
        STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa, 1013.25), rocket_data, 0);
        // Temperature
        STM32_i32To8((int32_t)bmp_data.temp_C, rocket_data, 4);
        // Roll Pitch
        STM32_i32To8((int32_t)icm_data.kalmanRoll, rocket_data, 8);
        STM32_i32To8((int32_t)icm_data.kalmanPitch, rocket_data, 12);
        // V_Batt
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), rocket_data, 20);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 22);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 24);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), rocket_data, 26);

        check = 1;
        break;
    case MODE_INFLIGHT:
        //BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);

        header_states = (rocket_data.header_states.mode << 6) | (rocket_data.header_states.pyro0 << 5) | (rocket_data.header_states.pyro1 << 4) | 0x00;

        rocket_data.size = INFLIGHT_DATASIZE;
        rocket_data.data = (uint8_t*)malloc(rocket_data.size * sizeof(uint8_t));
        // Altitude
        STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa, 1013.25), rocket_data, 0);
        // Temperature
        STM32_i32To8((int32_t)bmp_data.temp_C, rocket_data, 4);
        // GPS
        STM32_i32To8(L76_data.gps_data.time_raw, rocket_data, 8);
        STM32_i32To8(L76_data.gps_data.latitude, rocket_data, 12);
        //rocket_data.data[12] = gps_data.latitude_indicator;
        STM32_i32To8(L76_data.gps_data.longitude, rocket_data, 17);
        //rocket_data.data[17] = gps_data.longitude_indicator;
        //STM32_i32To8(gps_data.speed_knots, rocket_data, 22);
        //STM32_i32To8(gps_data.track_angle, rocket_data, 26);
        // Gyro
        STM32_i32To8((int32_t)icm_data.gyroX, rocket_data, 30);
        STM32_i32To8((int32_t)icm_data.gyroY, rocket_data, 34);
        STM32_i32To8((int32_t)icm_data.gyroZ, rocket_data, 38);
        // Acceleration
        STM32_i32To8((int32_t)icm_data.accX, rocket_data, 42);
        STM32_i32To8((int32_t)icm_data.accY, rocket_data, 46);
        STM32_i32To8((int32_t)icm_data.accZ, rocket_data, 50);
        // Roll Pitch
        STM32_i32To8((int32_t)icm_data.kalmanRoll, rocket_data, 54);
        STM32_i32To8((int32_t)icm_data.kalmanPitch, rocket_data, 58);

        check = 1;
        break;
    case MODE_POSTFLIGHT:
        //BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_LOW);

        header_states = (rocket_data.header_states.mode << 6) | 0x00;

        rocket_data.size = POSTFLIGHT_DATASIZE;
        rocket_data.data = (uint8_t*)malloc(rocket_data.size * sizeof(uint8_t));
        // Altitude
        STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa, 1013.25), rocket_data, 0);
        // GPS
        STM32_i32To8(L76_data.gps_data.time_raw, rocket_data, 8);
        STM32_i32To8(L76_data.gps_data.latitude, rocket_data, 12);
        //rocket_data.data[12] = gps_data.latitude_indicator;
        STM32_i32To8(L76_data.gps_data.longitude, rocket_data, 17);
        //rocket_data.data[17] = gps_data.longitude_indicator;
        //STM32_i32To8(gps_data.speed_knots, rocket_data, 22);
        //STM32_i32To8(gps_data.track_angle, rocket_data, 26);
        // V_Batt
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), rocket_data, 26);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 28);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 30);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), rocket_data, 32);

        check = 1;
        MEM2067_Unmount();
        break;
    case MODE_DEBUG:
    	// TODO: debug mode + wait ble connection
    	break;
    default:
        check = 0; // Error
    }

    rfd_data.header = header_states;
    rfd_data.size = rocket_data.size;
    rfd_data.data = rocket_data.data;
    // TODO: check CRC
    uint16_t crc = CRC16_Calculate(rocket_data.data, rocket_data.size);
    rocket_data.crc16[0] = (uint8_t)(crc >> 8);
    rocket_data.crc16[1] = (uint8_t)(crc & 0xFF);
    rfd_data.crc = (uint8_t*)rocket_data.crc16;

    if (RFD900_Send(&rfd_data) == 1)
        check = 1; // OK
    else
        check = 0; // ERROR

    free(rocket_data.data);
    return check;
}

uint8_t ROCKET_SetMode(uint8_t mode) {

    if (mode != MODE_PREFLIGHT && mode != MODE_INFLIGHT && mode != MODE_POSTFLIGHT) {
        return 0;
    }
    rocket_data.header_states.mode = mode;
    MEM2067_Write(filename_log, ROCKET_ModeToString(mode));
    return 1; // OK
}

bool Altitude_Trend(const float newAltitude) {

    bool isDescending = false;

    BMP280_buffer[bufferIndex] = newAltitude;
    bufferIndex = (bufferIndex + 1) % BMP280_BUFFERSIZE;

    // Check trend
    int descentDetected = 0;
    for (uint8_t i = 0; i < BMP280_BUFFERSIZE - 1; i++) {
        if (BMP280_buffer[i] > BMP280_buffer[i + 1]) {
            descentDetected++;
            if (descentDetected >= descentThreshold) {
                break;  // Stop the loop once threshold is reached
            }
        }
    }

    // Check sequence
    if (descentDetected >= descentThreshold) {
        descentCount++;
    } else {
        descentCount = 0;
    }
    if (descentCount >= descentThreshold) {
        isDescending = true;
    }

    return isDescending;
}


void STM32_u16To8(uint16_t data, ROCKET_Data rocket_data, uint8_t index) {

    rocket_data.data[index] = (uint8_t)(data >> 8);
    rocket_data.data[index + 1] = (uint8_t)(data & 0xFF);
}

void STM32_i32To8(int32_t data, ROCKET_Data rocket_data, uint8_t index) {

    rocket_data.data[index] = (uint8_t)((data >> 24) & 0xFF);
    rocket_data.data[index + 1] = (uint8_t)((data >> 16) & 0xFF);
    rocket_data.data[index + 2] = (uint8_t)((data >> 8) & 0xFF);
    rocket_data.data[index + 3] = (uint8_t)(data & 0xFF);
}

char* ROCKET_ModeToString(uint8_t mode) {

	switch(mode) {
	case 0x00: return "MODE_PREFLIGHT\r\n";
	case 0x01: return "MODE_INFLIGHT\r\n";
	case 0x02: return "MODE_POSTFLIGHT\r\n";
	default: return "Unknown mode\r\n";
	}
}

// Debugging
const char* ROCKET_BehaviorToString(uint8_t behavior) {
	//TODO: to do...
    switch (behavior) {
        case 0x00: return "Succeeded\r\n";
        case 0x01: return "A hard error occurred in the low level disk I/O layer\r\n";
        case 0x02: return "Assertion failed\r\n";
        case 0x03: return "The physical drive cannot work\r\n";
        case 0x04: return "Could not find the file\r\n";
        case 0x05: return "Could not find the path\r\n";
        case 0x06: return "The path name format is invalid\r\n";
        case 0x07: return "Access denied due to prohibited access or directory full\r\n";
        case 0x08: return "Access denied due to prohibited access\r\n";
        case 0x09: return "The file/directory object is invalid\r\n";
        case 0x0A: return "The physical drive is write protected\r\n";
        case 0x0B: return "The logical drive number is invalid\r\n";
        case 0x0C: return "The volume has no work area\r\n";
        case 0x0D: return "There is no valid FAT volume\r\n";
        case 0x0E: return "The f_mkfs() aborted due to any parameter error\r\n";
        case 0x0F: return "Could not get a grant to access the volume within defined period\r\n";
        case 0x10: return "The operation is rejected according to the file sharing policy\r\n";
        case 0x11: return "LFN working buffer could not be allocated\r\n";
        case 0x12: return "Number of open files > _FS_SHARE\r\n";
        case 0x13: return "Given parameter is invalid\r\n";
        default: return "Unknown rocket behavior\r\n";
    }
}

void RunTimerInit(RunTimer* dev) {

	  dev->start_time = HAL_GetTick();
	  dev->elapsed_time_ms = 0;
	  dev->elapsed_time_s = 0;
	  dev->elapsed_time_m = 0;
	  dev->elapsed_time_remaining_ms = 0;
}

void UpdateTime(RunTimer* dev) {

	dev->elapsed_time_ms = HAL_GetTick() - dev->start_time;

	// Convertir les millisecondes en secondes, minutes et millisecondes restantes
	dev->elapsed_time_s = (dev->elapsed_time_ms / 1000) % 60;
	dev->elapsed_time_m = (dev->elapsed_time_ms / 60000);
	dev->elapsed_time_remaining_ms = dev->elapsed_time_ms % 1000;
}

int printt(const char *format, ...) {

    va_list args;
    va_start(args, format);
    UpdateTime(&run_timer);
    printf("[%03d:%02d:%03d] ",run_timer.elapsed_time_m, run_timer.elapsed_time_s, run_timer.elapsed_time_remaining_ms);

    int ret = vprintf(format, args);

    va_end(args);
    return ret;
}

uint32_t square(int32_t p_Number)
{
	return p_Number * p_Number;
}


int _write(int le, char *ptr, int len) {

    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}
