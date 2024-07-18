/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"
#include "inttypes.h"

#include "GAUL_Drivers/WS2812_led.h"
#include "GAUL_Drivers/BMP280.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"
#include "GAUL_Drivers/Buzzer.h"
#include "GAUL_Drivers/RFD900.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/CRC_driver.h"
#include "GAUL_Drivers/L76LM33.h"
#include "GAUL_Drivers/HM10_BLE.h"
#include "GAUL_Drivers/CD74HC4051.h"
#include "GAUL_Drivers/Pyros.h"
#include "GAUL_Drivers/MEM2067.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	uint8_t mode;
	uint8_t pyro0;
	uint8_t pyro1;
	uint8_t accelerometer;
	uint8_t barometer;
	uint8_t gps;
	uint8_t sd;
	uint8_t ble;
} ROCKET_states;

typedef struct {
	ROCKET_states header_states;
	uint8_t *data;
	uint8_t crc16[2];
	uint8_t size;
} ROCKET_Packet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ICM_SPI_PORT 1
#define BMP_SPI_PORT 2
#define RFD_USART_PORT 1
#define GPS_USART_PORT 2
#define BT_USART_PORT  3

#define MODE_PREFLIGHT 0x00
#define PREFLIGHT_DATASIZE 28
#define MODE_INFLIGHT 0x01
#define INFLIGHT_DATASIZE 62
#define MODE_POSTFLIGHT 0x02
#define POSTFLIGHT_DATASIZE 34
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];
struct led_channel_info led_channels[WS2812_NUM_CHANNELS];

// Variables
GPS_Data gps_data;
BMP280 bmp_data;
ICM20602 icm_data;
RFD900 rfd_data;
HM10BLE ble_data;

uint8_t L76LM33_buffer[NMEA_TRAME_RMC_SIZE]; // gps_data buffer
uint8_t HM10BLE_buffer[20];  // ble_data buffer
ROCKET_Packet packet;

static const float accZMin = 1.1;		// delta > 0.9 g
static const float angleMin = 5;		// delta > 5 deg
static const float tempMin = 1;			// delta > 1 C
static const float pressMin = 10;		// delta > 10 Pa
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
void STM32_u16To8(uint16_t data, ROCKET_Packet packet, uint8_t index);
void STM32_i32To8(int32_t data, ROCKET_Packet packet, uint8_t index);
uint8_t ROCKET_SetMode(uint8_t mode);
void ROCKET_InitRoutine(void);
uint8_t ROCKET_ModeRoutine(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void STM32_u16To8(uint16_t data, ROCKET_Packet packet, uint8_t index) {

	packet.data[index] = (uint8_t)(data >> 8);
	packet.data[index + 1] = (uint8_t)(data & 0xFF);
}

void STM32_i32To8(int32_t data, ROCKET_Packet packet, uint8_t index) {

	packet.data[index] = (uint8_t)((data >> 24) & 0xFF);
	packet.data[index + 1] = (uint8_t)((data >> 16) & 0xFF);
	packet.data[index + 2] = (uint8_t)((data >> 8) & 0xFF);
	packet.data[index + 3] = (uint8_t)(data & 0xFF);
}

uint8_t ROCKET_SetMode(uint8_t mode) {

	if(mode != MODE_PREFLIGHT && mode != MODE_INFLIGHT && mode != MODE_POSTFLIGHT) {
		return 0;
	}
	packet.header_states.mode = mode;
	return 1; // OK
}

void ROCKET_InitRoutine(void) {

	/*
	HAL_Init();
	SystemClock_Config();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM3_Init();
	MX_TIM2_Init();
	MX_ADC1_Init();
	MX_CRC_Init();
	MX_FATFS_Init();
	*/

	printf("|----------Starting----------|\r\n");
	Buzz(&htim3, TIM_CHANNEL_4, START);
	SPI_Init(1);
	printf("(+) SPI1 succeeded...\r\n");
	SPI_Init(2);
	printf("(+) SPI2 succeeded...\r\n");
	USART_Init(1, 9600);
	printf("(+) USART1 succeeded...\r\n");
	USART_Init(2, 9600);
	printf("(+) USART2 succeeded...\r\n");
	USART_Init(3, 9600);
	printf("(+) USART3 succeeded...\r\n");

	printf("|----------Components initialization----------|\r\n");
	ROCKET_SetMode(MODE_PREFLIGHT);
	printf("(+) Mode flight: %i succeeded...\r\n", packet.header_states.mode);
	// LED RGB
	WS2812_Init();
	printf("(+) WS2812 succeeded...\r\n");
	// Multiplexer
	if (CD74HC4051_Init(&hadc1) != 1) {
	  printf("(-) CD74HC4051 failed...\r\n");
	} else {
		packet.header_states.pyro0 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_0, VREFPYRO);
		packet.header_states.pyro1 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_1, VREFPYRO);
		printf(" -> Pyro0 state: %i\r\n", packet.header_states.pyro0);
		printf(" -> Pyro1 state: %i\r\n", packet.header_states.pyro1);
		printf("(+) CD74HC4051 succeeded...\r\n");
	}
	// Barometer
	packet.header_states.barometer = BMP280_Init(&bmp_data, BMP_SPI_PORT) == 1 ? 0x01 : 0x00;
	printf(packet.header_states.barometer ? "(+) BMP280 succeeded...\r\n" : "(-) BMP280 failed...\r\n");
	// Accelerometer
	packet.header_states.accelerometer = ICM20602_Init(&icm_data) == 0 ? 0x01 : 0x00;
	printf(packet.header_states.accelerometer ? "(+) ICM20602 succeeded...\r\n" : "(-) ICM20602 failed...\r\n");
	// GPS
	packet.header_states.gps = L76LM33_Init(GPS_USART_PORT) == 1 ? 0x01 : 0x00;
	printf(packet.header_states.gps ? "(+) L76LM33 succeeded...\r\n" : "(-) L76LM33 failed...\r\n");
	// SD Card
	packet.header_states.sd = MEM2067_Mount("log.txt") == 1 ? 0x01 : 0x00;
	printf(packet.header_states.sd ? "(+) SD card succeeded...\r\n" : "(-) SD card failed...\r\n");
	MEM2067_Infos();
	// Bluetooth
	HM10BLE_Init(&ble_data, BT_USART_PORT);
}

uint8_t ROCKET_ModeRoutine(void) {

	uint8_t check = 0;

    uint8_t header_states = 0x00;
    packet.size = 0;
    packet.crc16[0] = 0x00;
    packet.crc16[1] = 0x00;

    switch (packet.header_states.mode) {
        case MODE_PREFLIGHT:
        	BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);
        	/*
        	if (HM10BLE_Connection(&ble_data, BT_USART_PORT, HM10BLE_buffer) == 1) {
				packet.header_states.ble = 0x01;
				printf("(+) HM10BLE connection succeeded...\r\n");
				printf(" -> En attente des valeurs de reference pour la temperature et de la pression(t;p)...\r\n");
				// TODO: Set ref values temp + press
			} else {
				packet.header_states.ble = 0x00;
			}
			*/
            header_states = (packet.header_states.mode << 6) | (packet.header_states.pyro0 << 5) | (packet.header_states.pyro1 << 4)
                            | (packet.header_states.accelerometer << 3) | (packet.header_states.barometer << 2) | (packet.header_states.gps << 1)
							| packet.header_states.sd;

            packet.size = PREFLIGHT_DATASIZE;
            packet.data = (uint8_t *)malloc(packet.size * sizeof(uint8_t));
            // Altitude
			STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa), packet, 0);
			 // Temperature
            STM32_i32To8((int32_t)bmp_data.temp_C, packet, 4);
            // Roll Pitch
            STM32_i32To8((int32_t)icm_data.kalmanAngleRoll, packet, 8);
            STM32_i32To8((int32_t)icm_data.kalmanAnglePitch, packet, 12);
            // V_Batt
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), packet, 20);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), packet, 22);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), packet, 24);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), packet, 26);

			check = 1;
            break;
        case MODE_INFLIGHT:
        	BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_NORMAL);

            header_states = (packet.header_states.mode << 6) | (packet.header_states.pyro0 << 5) | (packet.header_states.pyro1 << 4) | 0x00;

            packet.size = INFLIGHT_DATASIZE;
            packet.data = (uint8_t *)malloc(packet.size * sizeof(uint8_t));
            // Altitude
            STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa), packet, 0);
            // Temperature
            STM32_i32To8((int32_t)bmp_data.temp_C, packet, 4);
            // GPS
            STM32_i32To8(gps_data.time, packet, 8);
            STM32_i32To8(gps_data.latitude, packet, 12);
            packet.data[12] = gps_data.latitude_indicator;
            STM32_i32To8(gps_data.time, packet, 17);
            packet.data[17] = gps_data.longitude_indicator;
            STM32_i32To8(gps_data.speed_knots, packet, 22);
            STM32_i32To8(gps_data.track_angle, packet, 26);
            // Gyro
            STM32_i32To8((int32_t)icm_data.gyroX, packet, 30);
            STM32_i32To8((int32_t)icm_data.gyroY, packet, 34);
            STM32_i32To8((int32_t)icm_data.gyroZ, packet, 38);
            // Acceleration
            STM32_i32To8((int32_t)icm_data.accX, packet, 42);
            STM32_i32To8((int32_t)icm_data.accY, packet, 46);
            STM32_i32To8((int32_t)icm_data.accZ, packet, 50);
            // Roll Pitch
			STM32_i32To8((int32_t)icm_data.kalmanAngleRoll, packet, 54);
			STM32_i32To8((int32_t)icm_data.kalmanAnglePitch, packet, 58);

			check = 1;
            break;
        case MODE_POSTFLIGHT:
        	BMP280_SwapMode(BMP280_SETTING_CTRL_MEAS_LOW);
            header_states = (packet.header_states.mode << 6) | 0x00;

            packet.size = POSTFLIGHT_DATASIZE;
            packet.data = (uint8_t *)malloc(packet.size * sizeof(uint8_t));
            // Altitude
			STM32_i32To8((int32_t)BMP280_PressureToAltitude(bmp_data.pressure_Pa), packet, 0);
			// GPS
			STM32_i32To8(gps_data.time, packet, 4);
			STM32_i32To8(gps_data.latitude, packet, 8);
			packet.data[12] = gps_data.latitude_indicator;
			STM32_i32To8(gps_data.time, packet, 13);
			packet.data[17] = gps_data.longitude_indicator;
			STM32_i32To8(gps_data.speed_knots, packet, 18);
			STM32_i32To8(gps_data.track_angle, packet, 22);
			// V_Batt
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_3, PYRO_CHANNEL_DISABLED, VREFLIPO1), packet, 26);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_5, PYRO_CHANNEL_DISABLED, VREFLIPO3), packet, 28);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_2, PYRO_CHANNEL_DISABLED, VREFLIPO3), packet, 30);
			STM32_u16To8(CD74HC4051_AnRead(&hadc1, CHANNEL_6, PYRO_CHANNEL_DISABLED, VREF5VAN), packet, 32);

			check = 1;
            break;
        default:
        	check = 0; // Error
    }

    rfd_data.header = header_states;
    rfd_data.size = packet.size;
    rfd_data.data = packet.data;
    uint16_t crc = CRC16_Calculate(packet.data, packet.size);
    packet.crc16[0] = (uint8_t)(crc >> 8);
    packet.crc16[1] = (uint8_t)(crc & 0xFF);
    rfd_data.crc = (uint8_t *)packet.crc16;

    // TODO: add MEM2067_Write()

    if(RFD900_Send(&rfd_data, RFD_USART_PORT) == 1) {
    	free(packet.data);
    	check = 1; // OK
    } else {
    	free(packet.data);
    	check = 0; // ERROR
    }

    return check;
}

uint8_t ROCKET_Behavior(void) {

	uint8_t behavior = 0x00;
	// Variation
	/*
	accZ -> 1bit
	rollUp -> 2bits (4 etats)
	pitchUp -> 2bits (4 etats)
	temp -> 1bit
	press -> 1bit
	*/

	ICM20602_Update_All(&icm_data);
	BMP280_ReadTemperature(&bmp_data);
	BMP280_ReadPressure(&bmp_data);

	float old_temp = bmp_data.temp_C;
	float old_press = bmp_data.pressure_Pa;

	// Detect Z
	if(icm_data.accZ > 0) {
		behavior |= (1 << 0);	// up
	} else {
		behavior &= ~(1 << 0);	// down
	}
	if(icm_data.accZ <= accZMin && icm_data.accZ >= -accZMin) {
		behavior |= (1 << 1);	// idle z
		printf("Idle\n");
	} else{
		behavior &= ~(1 << 1);	// move z
	}
	// East
	if(icm_data.angleRoll >= angleMin) {
		if(icm_data.angleRoll <= 45) {
			behavior |= (1 << 2); // deviation +
		} else if(icm_data.angleRoll > 45) {
			behavior &= ~(1 << 2);	// deviation ++
		}
		printf("East: %0.1f\n", icm_data.angleRoll);
	} else {
		behavior |= (1 << 2);
	}
	// West
	if(icm_data.angleRoll <= -angleMin) {
		if(icm_data.angleRoll >= -45) {
			behavior |= (1 << 3); // deviation +
		} else if(icm_data.angleRoll < -45) {
			behavior &= ~(1 << 3);	// deviation +
		}
		printf("West: %0.1f\n", icm_data.angleRoll);
	} else {
		behavior |= (1 << 3);
	}
	// Sud
	if(icm_data.anglePitch <= -angleMin) {
		if(icm_data.anglePitch >= -45) {
			behavior |= (1 << 4); // deviation +
		} else if(icm_data.anglePitch < -45) {
			behavior &= ~(1 << 4);	// deviation ++
		}
		printf("South: %0.1f\n", icm_data.anglePitch);
	} else {
		behavior |= (1 << 4);
	}
	// Nord
	if(icm_data.anglePitch >= angleMin) {
		if(icm_data.anglePitch <= 45) {
			behavior |= (1 << 5); // deviation +
		} else if(icm_data.anglePitch > 45) {
			behavior &= ~(1 << 5);	// deviation ++
		}
		printf("North: %0.1f\n", icm_data.anglePitch);
	} else {
		behavior |= (1 << 5);
	}

	BMP280_ReadTemperature(&bmp_data);
	if(bmp_data.temp_C > old_temp + tempMin || bmp_data.temp_C < old_temp - tempMin) {
		behavior |= (1 << 6);
	} else {
		behavior &= ~(1 << 6);
	}
	BMP280_ReadPressure(&bmp_data);
	if(bmp_data.pressure_Pa > old_press + pressMin || bmp_data.pressure_Pa < old_press - pressMin) {
		behavior |= (1 << 7);
	} else {
		behavior &= ~(1 << 7);
	}

	return behavior;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_CRC_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 1 */
  ROCKET_InitRoutine();
  MEM2067_Unmount();
  /* USER CODE END 1 */

  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		// TODO: conditions flight mode change
		//ROCKET_Behavior();
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 14400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 127;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/* USER CODE BEGIN 4 */
int _write(int le, char *ptr, int len)
{
	int DataIdx;
	for(DataIdx = 0; DataIdx < len; DataIdx++)
	{
		ITM_SendChar(*ptr++);
	}
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
