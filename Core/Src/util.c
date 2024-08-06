/*
 * util.c
 *
 *  Created on: Aug 4, 2024
 *      Author: Luka
 */

#include "util.h"

// Handlers
extern ADC_HandleTypeDef hadc1;

// Structs
extern RunTimer run_timer;
extern GPS_Data gps_data;
extern ROCKET_Data rocket_data;
extern BMP280 bmp_data;
extern ICM20602 icm_data;
extern L76LM33 l76_data;
extern RFD900 rfd_data;
extern TIM_HandleTypeDef htim3;

// Buffers
float BMP280_buffer[BMP280_BUFFERSIZE]; // altimeter bmp

// Parameters
int bufferIndex = 0;
int descentCount = 0;
int descentThreshold = 5;
static const float accZMin = 1.1;		// delta > 0.9 g
static const float accResMin = 2.0;		// delta > 2.0 g
static const float angleMin = 5;		// delta > 5 deg

void ROCKET_InitRoutine(void)
{
    printt("|----------Starting----------|\r\n");
    RunTimerInit(&run_timer);
    Buzz(&htim3, TIM_CHANNEL_4, START);
    SPI_Init(SPI1);
    printt("(+) SPI1 succeeded...\r\n");
    SPI_Init(SPI2);
    printt("(+) SPI2 succeeded...\r\n");
    USART_Init(USART1);
    printt("(+) USART1 succeeded...\r\n");
    USART_Init(USART2);
    printt("(+) USART2 succeeded...\r\n");
    USART_Init(USART3);
    printt("(+) USART3 succeeded...\r\n");

    printt("|----------Components initialization----------|\r\n");
    ROCKET_SetMode(MODE_PREFLIGHT);
    printt("(+) Mode flight: %i succeeded...\r\n", rocket_data.header_states.mode);
    // LED RGB
    WS2812_Init();
    printt("(+) WS2812 succeeded...\r\n");
    // Multiplexer
    /*
     if (CD74HC4051_Init(&hadc1) != 1) {
     printt("(-) CD74HC4051 failed...\r\n");
     } else {
     rocket_data.header_states.pyro0 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_0, VREFPYRO);
     rocket_data.header_states.pyro1 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_1, VREFPYRO);
     printt(" -> Pyro0 state: %i\r\n", rocket_data.header_states.pyro0);
     printt(" -> Pyro1 state: %i\r\n", rocket_data.header_states.pyro1);
     printt("(+) CD74HC4051 succeeded...\r\n");
     }
     */
    // Barometer
    bmp_data.SPIx = SPI2;
    bmp_data.cs_pin = 8;
    bmp_data.cs_port = PA;
    rocket_data.header_states.barometer = BMP280_Init(&bmp_data) == 1 ? 0x01 : 0x00;
    printt(rocket_data.header_states.barometer ? "(+) BMP280 succeeded...\r\n" : "(-) BMP280 failed...\r\n");
    // Accelerometer
    icm_data.SPIx = SPI2;
    icm_data.cs_pin = 12;
    icm_data.cs_port = PB;
    icm_data.int_pin = 10;
    icm_data.int_port = PA;
    rocket_data.header_states.accelerometer = ICM20602_Init(&icm_data) == 1 ? 0x01 : 0x00;
    printt(rocket_data.header_states.accelerometer ? "(+) ICM20602 succeeded...\r\n" : "(-) ICM20602 failed...\r\n");
    // GPS
    l76_data.USARTx = USART2;
    rocket_data.header_states.gps = L76LM33_Init(&l76_data) == 1 ? 0x01 : 0x00;
    printt(rocket_data.header_states.gps ? "(+) L76LM33 succeeded...\r\n" : "(-) L76LM33 failed...\r\n");
    // Radio
    rfd_data.USARTx = USART1;
    rocket_data.header_states.rfd = RFD900_Init(&rfd_data) == 1 ? 0x01 : 0x00;
    printt(rocket_data.header_states.rfd ? "(+) RFD900 succeeded...\r\n" : "(-) RFD900 failed...\r\n");
    // SD Card
    rocket_data.header_states.sd = MEM2067_Mount("log.txt") == 1 ? 0x01 : 0x00;
    printt(rocket_data.header_states.sd ? "(+) SD card succeeded...\r\n" : "(-) SD card failed...\r\n");
    MEM2067_Infos();
    /*
     // Bluetooth
     HM10BLE_Init(&ble_data, BT_USART_PORT);
     */
    //TODO: MEM2067 Date + Task end time
}

uint8_t ROCKET_Behavior(void)
{

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
        printt("East: %0.1f\n", icm_data.angleRoll);
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
    if (icm_data.angleRoll >= angleMin) {
        behavior |= (1 << 2); // Detected
        printt("East: %0.1f\n", icm_data.angleRoll);
    } else {
        behavior &= ~(1 << 2); // Not detected
    }
    // West
    if (icm_data.angleRoll <= -angleMin) {
        behavior |= (1 << 3); // Detected
        printt("West: %0.1f\n", icm_data.angleRoll);
    } else {
        behavior &= ~(1 << 3); // Not detected
    }
    // South
    if (icm_data.anglePitch <= -angleMin) {
        behavior |= (1 << 4);
        printt("South: %0.1f\n", icm_data.anglePitch);
    } else {
        behavior &= ~(1 << 4);
    }
    // North
    if (icm_data.anglePitch >= angleMin) {
        behavior |= (1 << 5);
        printt("North: %0.1f\n", icm_data.anglePitch);
    } else {
        behavior &= ~(1 << 5);
    }
    // Altitude (buffer)
    if (Altitude_Trend(bmp_data.altitude_filtered_m) == true) {
        behavior |= (1 << 6);
        printt("Rocket is descending\n");
    } else {
        behavior &= ~(1 << 6);
    }
    // Mach Lock (filtered acceleration xyz)
    if (icm_data.accResult >= accResMin) {
        behavior |= (1 << 7);
        printt("Mach lock: Active\n");
    } else {
        behavior &= ~(1 << 7);
        printt("Mach lock: Disable\n");
    }

    return behavior;
}

uint8_t ROCKET_ModeRoutine(void)
{

    uint8_t check = 0;

    uint8_t header_states = 0x00;
    rocket_data.size = 0;
    rocket_data.crc16[0] = 0x00;
    rocket_data.crc16[1] = 0x00;

    switch (rocket_data.header_states.mode) {
    case MODE_PREFLIGHT:
        //BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);
        /*
         if (HM10BLE_Connection(&ble_data, BT_USART_PORT, HM10BLE_buffer) == 1) {
         rocket_data.header_states.ble = 0x01;
         printt("(+) HM10BLE connection succeeded...\r\n");
         printt(" -> En attente des valeurs de reference pour la temperature et de la pression(t;p)...\r\n");
         // TODO: Set ref values temp + press
         } else {
         rocket_data.header_states.ble = 0x00;
         }
         */
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
        STM32_i32To8((int32_t)icm_data.kalmanAngleRoll, rocket_data, 8);
        STM32_i32To8((int32_t)icm_data.kalmanAnglePitch, rocket_data, 12);
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
        STM32_i32To8(gps_data.time, rocket_data, 8);
        STM32_i32To8(gps_data.latitude, rocket_data, 12);
        rocket_data.data[12] = gps_data.latitude_indicator;
        STM32_i32To8(gps_data.time, rocket_data, 17);
        rocket_data.data[17] = gps_data.longitude_indicator;
        STM32_i32To8(gps_data.speed_knots, rocket_data, 22);
        STM32_i32To8(gps_data.track_angle, rocket_data, 26);
        // Gyro
        STM32_i32To8((int32_t)icm_data.gyroX, rocket_data, 30);
        STM32_i32To8((int32_t)icm_data.gyroY, rocket_data, 34);
        STM32_i32To8((int32_t)icm_data.gyroZ, rocket_data, 38);
        // Acceleration
        STM32_i32To8((int32_t)icm_data.accX, rocket_data, 42);
        STM32_i32To8((int32_t)icm_data.accY, rocket_data, 46);
        STM32_i32To8((int32_t)icm_data.accZ, rocket_data, 50);
        // Roll Pitch
        STM32_i32To8((int32_t)icm_data.kalmanAngleRoll, rocket_data, 54);
        STM32_i32To8((int32_t)icm_data.kalmanAnglePitch, rocket_data, 58);

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
        STM32_i32To8(gps_data.time, rocket_data, 4);
        STM32_i32To8(gps_data.latitude, rocket_data, 8);
        rocket_data.data[12] = gps_data.latitude_indicator;
        STM32_i32To8(gps_data.time, rocket_data, 13);
        rocket_data.data[17] = gps_data.longitude_indicator;
        STM32_i32To8(gps_data.speed_knots, rocket_data, 18);
        STM32_i32To8(gps_data.track_angle, rocket_data, 22);
        // V_Batt
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), rocket_data, 26);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 28);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), rocket_data, 30);
        STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), rocket_data, 32);

        check = 1;
        MEM2067_Unmount();
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

    // TODO: add MEM2067_Write()

    if (RFD900_Send(&rfd_data) == 1) {
        free(rocket_data.data);
        check = 1; // OK
    } else {
        free(rocket_data.data);
        check = 0; // ERROR
    }

    return check;
}

uint8_t ROCKET_SetMode(uint8_t mode)
{

    if (mode != MODE_PREFLIGHT && mode != MODE_INFLIGHT && mode != MODE_POSTFLIGHT) {
        return 0;
    }
    rocket_data.header_states.mode = mode;
    return 1; // OK
}

bool Altitude_Trend(const float newAltitude)
{

    bool isDescending = false;

    BMP280_buffer[bufferIndex] = newAltitude;
    bufferIndex = (bufferIndex + 1) % BMP280_BUFFERSIZE;

    // Check trend
    int descentDetected = 0;
    for (uint8_t i = 0; i < BMP280_BUFFERSIZE - 1; i++) {
        if (BMP280_buffer[i] > BMP280_buffer[i + 1]) {
            descentDetected++;
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
    } else {
        isDescending = false;
    }

    return isDescending;
}

void STM32_u16To8(uint16_t data, ROCKET_Data rocket_data, uint8_t index)
{

    rocket_data.data[index] = (uint8_t)(data >> 8);
    rocket_data.data[index + 1] = (uint8_t)(data & 0xFF);
}

void STM32_i32To8(int32_t data, ROCKET_Data rocket_data, uint8_t index)
{

    rocket_data.data[index] = (uint8_t)((data >> 24) & 0xFF);
    rocket_data.data[index + 1] = (uint8_t)((data >> 16) & 0xFF);
    rocket_data.data[index + 2] = (uint8_t)((data >> 8) & 0xFF);
    rocket_data.data[index + 3] = (uint8_t)(data & 0xFF);
}

// Debugging
const char* ROCKET_BehaviorToString(uint8_t behavior)
{

    switch (behavior) {
    /*
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
     */
    default:
        return "Unknown rocket behavior\r\n";
    }
}

void RunTimerInit(RunTimer* dev)
{
	  dev->start_time = HAL_GetTick();
	  dev->elapsed_time_ms = 0;
	  dev->elapsed_time_s = 0;
	  dev->elapsed_time_m = 0;
	  dev->elapsed_time_remaining_ms = 0;
}

void UpdateTime(RunTimer* dev)
{
	dev->elapsed_time_ms = HAL_GetTick() - dev->start_time;

	// Convertir les millisecondes en secondes, minutes et millisecondes restantes
	dev->elapsed_time_s = (dev->elapsed_time_ms / 1000) % 60;
	dev->elapsed_time_m = (dev->elapsed_time_ms / 60000);
	dev->elapsed_time_remaining_ms = dev->elapsed_time_ms % 1000;
}

int printt(const char *format, ...) {
    va_list args;
    va_start(args, format);

    // Ajouter des fonctionnalités supplémentaires ici
    // Par exemple, vous pouvez ajouter un timestamp avant chaque message
    printf("[%03d:%02d:%03d] ",run_timer.elapsed_time_m, run_timer.elapsed_time_s, run_timer.elapsed_time_remaining_ms);

    // Appeler la fonction printf originale avec les arguments
    int ret = vprintf(format, args);

    va_end(args);
    return ret;
}

int _write(int le, char *ptr, int len) {
    int DataIdx;
    for (DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}
