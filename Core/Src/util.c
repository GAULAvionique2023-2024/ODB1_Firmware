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
extern HM10BLE ble_data;

// Buffers
static uint8_t rocket_data_buffer[MAX_ROCKET_DATA_SIZE];
static char u_char_buffer[128] = {"0"};
// TODO: add hm10 response buffer

// Parameters
static uint8_t header_states = 0x00;

// Variable
//extern bool push_button;

void ROCKET_InitRoutine(void) {

	printt("|----------Starting----------|\r\n");
	RunTimerInit(&run_timer);
	//Buzz(TIM3, LL_TIM_CHANNEL_CH4, START);
	SPI_Init(SPI1);
	SPI_Init(SPI2);
//	USART_Init(USART1, 9600, 72);
//	USART_Init(USART2, 9600, 72);
//	USART_Init(USART3, 9600, 72);
	printt("|----------Components initialization----------|\r\n");
	// Button
//	Init_Interrupt_GPIO(GPIOA, 9);
	ROCKET_SetMode(MODE_PREFLIGHT);
	printt("(+) Mode flight: %i succeeded...\r\n", rocket_data.header_states.mode);
	// LED RGB
//	WS2812_Init();
//	printt("(+) WS2812 succeeded...\r\n");
	// Multiplexer
	if (CD74HC4051_Init(&hadc1) != 1) {
	  printt("(-) CD74HC4051 failed...\r\n");
	} else {
		rocket_data.header_states.pyro0 = Pyro_Check(&hadc1, PYRO_CHANNEL_0) ? 0x01 : 0x00;
		rocket_data.header_states.pyro1 = Pyro_Check(&hadc1, PYRO_CHANNEL_1) ? 0x01 : 0x00;
		printt(" -> Pyro0 state: %i\r\n", rocket_data.header_states.pyro0);
		printt(" -> Pyro1 state: %i\r\n", rocket_data.header_states.pyro1);
		printt("(+) CD74HC4051 succeeded...\r\n");
	}
	// Barometer
	bmp_data.SPIx = SPI2;
	bmp_data.cs_pin = 8;
	bmp_data.cs_port = GPIOA;
	rocket_data.header_states.barometer = BMP280_Init(&bmp_data) == 0 ? 0x01 : 0x00;
	printt(rocket_data.header_states.barometer ? "(+) BMP280 succeeded...\r\n" : "(-) BMP280 failed...\r\n");
	// Accelerometer
	icm_data.SPIx = SPI2;
	icm_data.cs_pin = 12;
	icm_data.cs_port = GPIOB;
	icm_data.int_pin = 10;
	icm_data.int_port = GPIOA;
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
	rocket_data.header_states.sd = MEM2067_Mount(FILENAME_LOG) == 1 ? 0x01 : 0x00;
	printt(rocket_data.header_states.sd ? "(+) SD card succeeded...\r\n" : "(-) SD card failed...\r\n");
	// Bluetooth
//	ble_data.USARTx = USART3;
//	HM10BLE_Init(&ble_data);

	// Write LOG data
	ParseTimerBuffer(&run_timer, u_char_buffer);
	MEM2067_Write(FILENAME_LOG, u_char_buffer);
	MEM2067_Write(FILENAME_LOG, "\r\n");
}

uint8_t ROCKET_Behavior(void) {

    ICM20602_Update_All(&icm_data);
    BMP280_Read_Temperature_Pressure(&bmp_data);

    // Not in mach lock (engine not burning)
    if (icm_data.accZ <= ACCZ_MIN && icm_data.accZ >= -ACCZ_MIN) {}

//    uint8_t behavior = 0x00;
//    // Orientation Z
//    if (icm_data.accZ > 0) {
//        behavior |= (1 << 0);	// up
//    } else behavior &= ~(1 << 0);	// down
//    // Movement not in mach lock
//    if (icm_data.accZ <= ACCZ_MIN && icm_data.accZ >= -ACCZ_MIN) {
//    	AltitudeTrend trend = Altitude_Trend(bmp_data.altitude_filtered_m);
//    	if(trend == ASCENDING) {
//    		behavior |= (1 << 1);
//    		behavior |= (0 << 2);
//    	} else if(trend == DESCENDING) {
//    		behavior |= (0 << 1);
//			behavior |= (1 << 2);
//    	} else {
//    		behavior |= (0 << 1);
//			behavior |= (0 << 2);
//    	} // No 0x03
//    }
//    // East
//    if (icm_data.angleX >= ANGLE_MIN) {
//        behavior |= (1 << 3); // Detected
//    } else behavior &= ~(1 << 3); // Not detected
//    // West
//    if (icm_data.angleX <= -ANGLE_MIN) {
//        behavior |= (1 << 4); // Detected
//    } else behavior &= ~(1 << 4); // Not detected
//    // South
//    if (icm_data.angleY <= -ANGLE_MIN) {
//        behavior |= (1 << 5);
//    } else behavior &= ~(1 << 5);
//    // North
//    if (icm_data.angleY >= ANGLE_MIN) {
//        behavior |= (1 << 6);
//    } else behavior &= ~(1 << 6);
//    // Mach Lock (vector norm acceleration)
//    if (icm_data.accResult >= ACCRES_MIN) {
//        behavior |= (1 << 7);
//    } else behavior &= ~(1 << 7);
//
//    return behavior;
    return 0; // TMP Launch Canada
}

uint8_t ROCKET_ModeRoutine(void) {

    uint8_t check = 0;

    // Reset rocket data
	rocket_data.size = 0;
	memset(rocket_data.crc16, 0, sizeof(rocket_data.crc16));
	rocket_data.data = rocket_data_buffer;

	L76LM33_Read(&L76_data);

	// Write LOG timer
	ParseTimerBuffer(&run_timer, u_char_buffer);
	MEM2067_Write(FILENAME_LOG, u_char_buffer);

	// Set const variable
	rocket_data.header_states.pyro0 = Pyro_Check(&hadc1, PYRO_CHANNEL_0);
	rocket_data.header_states.pyro1 = Pyro_Check(&hadc1, PYRO_CHANNEL_1);
	rocket_data.header_states.gps_fix = L76_data.gps_data.fix;
	header_states = (rocket_data.header_states.mode << 6)
			| (rocket_data.header_states.pyro0 << 5)
			| (rocket_data.header_states.pyro1 << 4)
			| (rocket_data.header_states.accelerometer << 3)
			| (rocket_data.header_states.barometer << 2)
			| (rocket_data.header_states.gps_fix << 1)
			| (rocket_data.header_states.sd);
	// Write LOG states
	sprintf(u_char_buffer, "%u\r\n", header_states);
	MEM2067_Write(FILENAME_LOG, u_char_buffer);
	MEM2067_Write(FILENAME_LOG, "  |  ");

//	// Debug mode
//	if(push_button == true) {
//		//printf("Debug mode enabled...\r\n");
//	} else {
//		//printf("Debug mode disabled...\r\n");
//	}

    switch (rocket_data.header_states.mode) {
		case MODE_PREFLIGHT:
			//BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);
			rocket_data.size = PREFLIGHT_DATASIZE;

			// Altitude
			STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa, 1013.25), rocket_data, 0);
			// Temperature
			STM32_i32To8((int32_t)bmp_data.temp_C, rocket_data, 4);
			// Roll Pitch
			STM32_i32To8((int32_t)icm_data.kalmanRoll, rocket_data, 8);
			STM32_i32To8((int32_t)icm_data.kalmanPitch, rocket_data, 12);
			// V_Batt
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), rocket_data, 16);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 18);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 20);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), rocket_data, 22);

			check = 1;
			break;
		case MODE_INFLIGHT:
			//BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);
//			rocket_data.size = INFLIGHT_DATASIZE;
			rocket_data.size = 48;

			// Altitude
			STM32_fTo8(bmp_data.altitude_filtered_m, rocket_data, 0);
			// Temperature
			STM32_fTo8(bmp_data.temp_C, rocket_data, 4);
			// GPS
//			STM32_i32To8(L76_data.gps_data.time_raw, rocket_data, 8);
			STM32_fTo8(L76_data.gps_data.latitude, rocket_data, 8);
			STM32_fTo8(L76_data.gps_data.longitude, rocket_data, 12);
			// Gyro
			STM32_fTo8(icm_data.gyroX, rocket_data, 16);
			STM32_fTo8(icm_data.gyroY, rocket_data, 20);
			STM32_fTo8(icm_data.gyroZ, rocket_data, 24);
			// Acceleration
			STM32_fTo8(icm_data.accX, rocket_data, 28);
			STM32_fTo8(icm_data.accY, rocket_data, 32);
			STM32_fTo8(icm_data.accZ, rocket_data, 36);
			// Roll Pitch
			STM32_fTo8(icm_data.kalmanRoll, rocket_data, 40);
			STM32_fTo8(icm_data.kalmanPitch, rocket_data, 44);
			// V_Batt
			// TODO
//			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), rocket_data, 16);
//			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 18);
//			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 20);

			check = 1;
			break;
		case MODE_POSTFLIGHT:
			//BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_LOW);
			rocket_data.size = POSTFLIGHT_DATASIZE;

			// Altitude
			STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa, 1013.25), rocket_data, 0);
			// GPS
			STM32_i32To8(L76_data.gps_data.time_raw, rocket_data, 4);
			STM32_i32To8(L76_data.gps_data.latitude, rocket_data, 8);
			STM32_i32To8(L76_data.gps_data.longitude, rocket_data, 12);
			// V_Batt
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), rocket_data, 16);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 18);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 20);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), rocket_data, 22);

			check = 1;
			// TODO: add condition execute one time in loop
			MEM2067_Unmount();
			break;
//		case MODE_DEBUG:
//			// TODO: debug mode + wait ble connection
//			break;
		default:
			check = 0; // Error
    }

    // Write LOG data
    // TODO: fix mem2067 data write (uint8_t* -> char*)
    sprintf(u_char_buffer, "%u\r\n", rocket_data.data);
    MEM2067_Write(FILENAME_LOG, u_char_buffer);

    rfd_data.header = header_states;
    rfd_data.size = rocket_data.size;
    rfd_data.data = rocket_data.data;

    uint16_t crc = CRC16_Calculate(rocket_data.data, rocket_data.size);
    rocket_data.crc16[0] = (uint8_t)(crc >> 8);
    rocket_data.crc16[1] = (uint8_t)(crc & 0xFF);
    rfd_data.crc = (uint8_t*)rocket_data.crc16;

    if (RFD900_Send(&rfd_data) == 1)
        check = 1; // OK
    else
        check = 0; // ERROR

    return check;
}

uint8_t ROCKET_SetMode(const uint8_t mode) {

    if (mode != MODE_PREFLIGHT && mode != MODE_INFLIGHT && mode != MODE_POSTFLIGHT && mode != MODE_DEBUG) {
        return 0;
    }

    if(rocket_data.header_states.mode != mode) {
		rocket_data.header_states.mode = mode;
		MEM2067_Write(FILENAME_LOG, ROCKET_ModeToString(mode));
    }
    return 1; // OK
}

AltitudeTrend Altitude_Trend(const float newAltitude) {

    static uint8_t bufferIndex = 0;
    static uint8_t ascentCount = 0;
    static uint8_t descentCount = 0;
    static float BMP280_buffer[BMP280_BUFFERSIZE] = {0};

    BMP280_buffer[bufferIndex] = newAltitude;
    bufferIndex = (bufferIndex + 1) % BMP280_BUFFERSIZE;

    uint8_t ascentDetected = 0;
    uint8_t descentDetected = 0;

    for (uint8_t i = 0; i < BMP280_BUFFERSIZE - 1; i++) {
        if (BMP280_buffer[i] < BMP280_buffer[i + 1]) {
            ascentDetected++;
        } else if (BMP280_buffer[i] > BMP280_buffer[i + 1]) {
            descentDetected++;
        }
    }

    ascentCount = (ascentDetected >= ALTITUDE_TREND_THRESHOLD) ? ascentCount + 1 : 0;
    descentCount = (descentDetected >= ALTITUDE_TREND_THRESHOLD) ? descentCount + 1 : 0;

    if (ascentCount >= ALTITUDE_TREND_THRESHOLD) {
        return ASCENDING;
    } else if (descentCount >= ALTITUDE_TREND_THRESHOLD) {
        return DESCENDING;
    } else {
        return STABLE;
    }
}

static void writeBytes(uint8_t* dest, uint32_t data, uint8_t size) {

    for (uint8_t i = 0; i < size; i++) {
        dest[i] = (uint8_t)((data >> ((size - i - 1) * 8)) & 0xFF);
    }
}

void STM32_u16To8(uint16_t data, ROCKET_Data rocket_data, uint8_t index) {

    writeBytes(&rocket_data.data[index], data, 2);
}

void STM32_i32To8(int32_t data, ROCKET_Data rocket_data, uint8_t index) {

    writeBytes(&rocket_data.data[index], (uint32_t)data, 4);
}

void STM32_fTo8(float data, ROCKET_Data rocket_data, uint8_t index) {
    uint32_t asInt = *((int*)&data);

    uint8_t i;
    for (i = 0; i < 4; i++) {
    	rocket_data.data[index + i] = (uint8_t)((asInt >> 8 * i) & 0xFF);
    }
}

const char* ROCKET_ModeToString(const uint8_t mode) {

	switch(mode) {
		case 0x00: return "MODE_PREFLIGHT\r\n";
		case 0x01: return "MODE_INFLIGHT\r\n";
		case 0x02: return "MODE_POSTFLIGHT\r\n";
		case 0x03: return "MODE_DEBUG\r\n";
		default: return "Unknown mode\r\n";
	}
}

// Debugging
const char* ROCKET_BehaviorToString(const uint8_t behavior) {

    switch (behavior) {
        case 0x00: return "Rocket is idle\r\n"; // No significant behavior detected
        case 0x01: return "Rocket is moving in Z direction (Up)\r\n"; // Moving up in Z direction
        case 0x02: return "Rocket is moving in Z direction (Down)\r\n"; // Moving down in Z direction
        case 0x03: return "Rocket is moving in Z direction (Idle)\r\n"; // Idle in Z direction
        case 0x04: return "Rocket is moving East\r\n"; // Moving East
        case 0x05: return "Rocket is moving West\r\n"; // Moving West
        case 0x06: return "Rocket is moving South\r\n"; // Moving South
        case 0x07: return "Rocket is moving North\r\n"; // Moving North
        case 0x08: return "Rocket is descending\r\n"; // Altitude trend indicates descent
        case 0x09: return "Mach lock is active\r\n"; // Mach lock is active
        case 0x0A: return "Rocket is moving East and Mach lock is active\r\n"; // East and Mach lock active
        case 0x0B: return "Rocket is moving West and Mach lock is active\r\n"; // West and Mach lock active
        case 0x0C: return "Rocket is moving South and Mach lock is active\r\n"; // South and Mach lock active
        case 0x0D: return "Rocket is moving North and Mach lock is active\r\n"; // North and Mach lock active
        case 0x0E: return "Rocket is descending and Mach lock is active\r\n"; // Descending and Mach lock active
        case 0x0F: return "Mach lock is disabled\r\n"; // Mach lock is disabled
        case 0x10: return "Rocket is moving East and Mach lock is disabled\r\n"; // East and Mach lock disabled
        case 0x11: return "Rocket is moving West and Mach lock is disabled\r\n"; // West and Mach lock disabled
        case 0x12: return "Rocket is moving South and Mach lock is disabled\r\n"; // South and Mach lock disabled
        case 0x13: return "Rocket is moving North and Mach lock is disabled\r\n"; // North and Mach lock disabled
        case 0x14: return "Rocket is descending and Mach lock is disabled\r\n"; // Descending and Mach lock disabled
        default: return "Unknown rocket behavior\r\n"; // Unknown code
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

void ParseTimerBuffer(RunTimer* dev, char *buffer) {

	UpdateTime(dev);
	sprintf(buffer, "[%03d:%02d:%03d] : ",dev->elapsed_time_m, dev->elapsed_time_s, dev->elapsed_time_remaining_ms);
}

uint32_t square(int32_t p_Number) {

	return p_Number * p_Number;
}


int _write(int le, char *ptr, int len) {

    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}
