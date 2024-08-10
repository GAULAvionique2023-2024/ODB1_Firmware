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
//extern MEM2067 mem_data;
extern TIM_HandleTypeDef htim3;

// Definitions
#define MAX_ROCKET_DATA_SIZE (INFLIGHT_DATASIZE > POSTFLIGHT_DATASIZE ? INFLIGHT_DATASIZE : POSTFLIGHT_DATASIZE)
#define DESCENT_THRESHOLD 5
#define ACCZ_MIN 1.1
#define ACCRES_MIN 2.0
#define ANGLE_MIN 5

// Buffers
static uint8_t rocket_data_buffer[MAX_ROCKET_DATA_SIZE];
static float BMP280_buffer[BMP280_BUFFERSIZE]; // altimeter bmp
// TODO: add hm10 response buffer

// Parameters
static uint8_t header_states = 0x00;

//extern bool isButton_push = false;

void ROCKET_InitRoutine(void) {

	printt("|----------Starting----------|\r\n");
	RunTimerInit(&run_timer);
	Buzz(&htim3, TIM_CHANNEL_4, START);
	SPI_Init(SPI1);
	SPI_Init(SPI2);
	USART_Init(USART1);
	USART_Init(USART2);
	USART_Init(USART3);
	ROCKET_SetMode(MODE_PREFLIGHT);
	// LED RGB
	WS2812_Init();
	// Multiplexer
	if (CD74HC4051_Init(&hadc1) == 1) {
		rocket_data.header_states.pyro0 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_0, VREFPYRO);
		rocket_data.header_states.pyro1 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_1, VREFPYRO);
	}
	// Barometer
	bmp_data.SPIx = SPI2;
	bmp_data.cs_pin = 8;
	bmp_data.cs_port = PA;
	rocket_data.header_states.barometer = BMP280_Init(&bmp_data) == 0 ? 0x01 : 0x00;
	// Accelerometer
	icm_data.SPIx = SPI2;
	icm_data.cs_pin = 12;
	icm_data.cs_port = PB;
	icm_data.int_pin = 10;
	icm_data.int_port = PA;
	rocket_data.header_states.accelerometer = ICM20602_Init(&icm_data) == 0 ? 0x01 : 0x00;
	// GPS
	rocket_data.header_states.gps = L76LM33_Init(&L76_data, &huart2) == L76LM33_OK ? 0x01 : 0x00;
	// Radio
	rfd_data.USARTx = USART1;
	rocket_data.header_states.rfd = RFD900_Init(&rfd_data) == 1 ? 0x01 : 0x00;
	// SD Card
	rocket_data.header_states.sd = MEM2067_Mount(FILENAME_LOG) == 1 ? 0x01 : 0x00;
	// Bluetooth
	ble_data.USARTx = USART3;
	HM10BLE_Init(&ble_data);
	/*
	char time[20];
	itoa(L76_data.gps_data.time_raw, time, 10);
	MEM2067_Write(FILENAME_LOG, time);
	*/
	// Set const variable
	header_states = (rocket_data.header_states.mode << 6) | (rocket_data.header_states.pyro0 << 5) | (rocket_data.header_states.pyro1 << 4)
					| (rocket_data.header_states.accelerometer << 3) | (rocket_data.header_states.barometer << 2) | (rocket_data.header_states.gps << 1)
					| rocket_data.header_states.sd;
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
    } else {
        behavior &= ~(1 << 0);	// down
    }
    // Movement
    if (icm_data.accZ <= ACCZ_MIN && icm_data.accZ >= -ACCZ_MIN) {
        behavior |= (1 << 1);	// idle z
    } else {
        behavior &= ~(1 << 1);	// move z
    }
    // East
    if (icm_data.angleX >= ANGLE_MIN) {
        behavior |= (1 << 2); // Detected
    } else {
        behavior &= ~(1 << 2); // Not detected
    }
    // West
    if (icm_data.angleX <= -ANGLE_MIN) {
        behavior |= (1 << 3); // Detected
    } else {
        behavior &= ~(1 << 3); // Not detected
    }
    // South
    if (icm_data.angleY <= -ANGLE_MIN) {
        behavior |= (1 << 4);
    } else {
        behavior &= ~(1 << 4);
    }
    // North
    if (icm_data.angleY >= ANGLE_MIN) {
        behavior |= (1 << 5);
    } else {
        behavior &= ~(1 << 5);
    }
    // Altitude (falling)
    if (Altitude_Trend(bmp_data.altitude_filtered_m) == true) {
        behavior |= (1 << 6);
    } else {
        behavior &= ~(1 << 6);
    }
    // Mach Lock (vector norm acceleration)
    if (icm_data.accResult >= ACCRES_MIN) {
        behavior |= (1 << 7);
    } else {
        behavior &= ~(1 << 7);
    }

    return behavior;
}

uint8_t ROCKET_ModeRoutine(void) {

    uint8_t check = 0;

    // Reset rocket data
	rocket_data.size = 0;
	memset(rocket_data.crc16, 0, sizeof(rocket_data.crc16));
	rocket_data.data = rocket_data_buffer;
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
			rocket_data.size = PREFLIGHT_DATASIZE;

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
			rocket_data.size = INFLIGHT_DATASIZE;

			// Altitude
			STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa, 1013.25), rocket_data, 0);
			// Temperature
			STM32_i32To8((int32_t)bmp_data.temp_C, rocket_data, 4);
			// GPS
			STM32_i32To8(L76_data.gps_data.time_raw, rocket_data, 8);
			STM32_i32To8(L76_data.gps_data.latitude, rocket_data, 12);
			STM32_i32To8(L76_data.gps_data.longitude, rocket_data, 16);
			// Gyro
			STM32_i32To8((int32_t)icm_data.gyroX, rocket_data, 20);
			STM32_i32To8((int32_t)icm_data.gyroY, rocket_data, 24);
			STM32_i32To8((int32_t)icm_data.gyroZ, rocket_data, 28);
			// Acceleration
			STM32_i32To8((int32_t)icm_data.accX, rocket_data, 32);
			STM32_i32To8((int32_t)icm_data.accY, rocket_data, 36);
			STM32_i32To8((int32_t)icm_data.accZ, rocket_data, 40);
			// Roll Pitch
			STM32_i32To8((int32_t)icm_data.kalmanRoll, rocket_data, 44);
			STM32_i32To8((int32_t)icm_data.kalmanPitch, rocket_data, 48);

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

			check = 1;
			// TODO: add condition execute one time in loop
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
    rocket_data.header_states.mode = mode;
    MEM2067_Write(FILENAME_LOG, ROCKET_ModeToString(mode));
    return 1; // OK
}

bool Altitude_Trend(const float newAltitude) {

	uint8_t bufferIndex = 0;
	uint8_t descentCount = 0;

    BMP280_buffer[bufferIndex] = newAltitude;
    bufferIndex = (bufferIndex + 1) % BMP280_BUFFERSIZE;

    uint8_t descentDetected = 0;
    bool isDescending = false;

    for (uint8_t i = 0; i < BMP280_BUFFERSIZE - 1; i++) {
        if (BMP280_buffer[i] > BMP280_buffer[i + 1]) {
            descentDetected++;
            if (descentDetected >= DESCENT_THRESHOLD) {
                break;
            }
        }
    }

    descentCount = (descentDetected >= DESCENT_THRESHOLD) ? descentCount + 1 : 0;
    if (descentCount >= DESCENT_THRESHOLD) {
        isDescending = true;
    }

    return isDescending;
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
