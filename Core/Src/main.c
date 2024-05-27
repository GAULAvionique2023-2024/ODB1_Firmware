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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

#include "GAUL_Drivers/WS2812_led.h"
#include "GAUL_Drivers/BMP280.h"
#include "GAUL_Drivers/ICM20602.h"
#include "GAUL_Drivers/Low_Level_Drivers/GPIO_driver.h"
#include "GAUL_Drivers/Low_Level_Drivers/SPI_driver.h"
#include "GAUL_Drivers/Buzzer.h"
#include "GAUL_Drivers/RFD900.h"
#include "GAUL_Drivers/Low_Level_Drivers/USART_driver.h"
#include "GAUL_Drivers/L76LM33.h"
#include "GAUL_Drivers/HM10_BLE.h"
#include "GAUL_Drivers/CD74HC4051.h"
#include "GAUL_Drivers/Pyros.h"
#include "GAUL_Drivers/MEM2067.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// $Stats(8);V_Batt_Lipo1(16);V_Batt_Lipo2(16);V_Batt_Lipo3(16);5V_AN(16);Temp(32);Altitude(16);AngleRoll(32);AnglePitch(32)*CRC(16)$ => (28)
#define MODE_PREFLIGHT 0x00
#define PREFLIGHT_SIZE 28 // mode = 0x00
// $Stats(8);GPS_Data(416);Gyro_Data(72)*CRC(16)$ => (64)
#define MODE_FLIGHT 0x01
#define FLIGHT_SIZE 64 // mode = 0x01
// $Stats(8);GPS_Data(296);V_Batt_Lipo1(16);V_Batt_Lipo2(16);V_Batt_Lipo3(16);5V_AN(16) => (46)
#define MODE_POSTFLIGHT 0x02
#define POSTFLIGHT_SIZE 46 // mode = 0x02
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
CRC_HandleTypeDef hcrc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];
struct led_channel_info led_channels[WS2812_NUM_CHANNELS];

// Variables
GPS_Data gps_data;
BMP280 bmp_data;
ICM20602 icm_data;
RFD900 rfd_data;
HM10BLE ble_data;
char L76LM33_Buffer[NMEA_TRAME_RMC_SIZE]; // gps_data buffer

typedef struct {
	uint8_t mode;
	uint8_t pyro1;
	uint8_t pyro2;
	uint8_t accelerometer;
	uint8_t barometer;
	uint8_t gps;
	uint8_t sd;
} STM32_states;
STM32_states states;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void STM32_InitRoutine(void) {

	/* MCU Configuration--------------------------------------------------------*/
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

	/* Communications Configuration-------------------------------------------------------- */
	printf("|----------Starting----------|\r\n");
	SPI_Init(1);
	printf("(+) SPI1 succeeded...\r\n");
	SPI_Init(2);
	printf("(+) SPI2 succeeded...\r\n");
	USART_Init(1);
	printf("(+) USART1 succeeded...\r\n");
	USART_Init(2);
	printf("(+) USART2 succeeded...\r\n");
	USART_Init(3);
	printf("(+) USART3 succeeded...\r\n");

	/* Components Configuration-------------------------------------------------------- */
	printf("|----------Components initialization----------|\r\n");
	// LED RGB
	WS2812_Init();
	printf("(+) WS2812 succeeded...\r\n");
	// Multiplexer
	if (CD74HC4051_Init(&hadc1) != 0) {
	  printf("(-) CD74HC4051 failed...\r\n");
	} else {
	  printf("(+) CD74HC4051 succeeded...\r\n");
	}
	// Barometer
	if (BMP280_Init(&bmp_data) != 0) {
	  printf("(-) BMP280 failed...\r\n");
	} else {
	  printf("(+) BMP280 succeeded...\r\n");
	}
	// Accelerometer
	if (ICM20602_Init(&icm_data) != 0) {
	  printf("(-) ICM20602 failed...\r\n");
	} else {
	  printf("(+) ICM20602 succeeded...\r\n");
	}
	// GPS
	if(L76LM33_Init() == 1) {
	  printf("(-) L76LM33 failed...\r\n");
	} else {
	  printf("(+) L76LM33 succeeded...\r\n");
	}
	// SD Card
	if (MEM2067_SDCardDetection() == 1) {
	  printf("(-) No SD card detected in MEM2067...\r\n");
	} else {
	  printf("(+) SD card detected in MEM2067...\r\n");
	}
	/* USER CODE END 2 */

	// TESTS
	/*
	// Test d'écriture
	const char *filename = "test.txt";
	uint8_t data[] = "Hello, SD Card!";
	uint16_t size = sizeof(data) - 1;
	if (MEM2067_WriteFATFS(filename, data, size) == 0) {
	  printf("(+) Writing to %s succeeded...\r\n", filename);
	} else {
	  printf("(-) Writing %s failed...\r\n", filename);
	}

	while (1)
	{
		//Test GPS
	  memset(L76LM33_Buffer, 0, NMEA_TRAME_RMC_SIZE);
	  USART_RX(2, (uint8_t*)L76LM33_Buffer, NMEA_TRAME_RMC_SIZE);
	  // Affichage de la trame NMEA reçue
	  printf("NMEA sentence: %s", L76LM33_Buffer);
	  NMEA_Decode_GPRMC(L76LM33_Buffer, &gps_data);
	  printf("Time: %s\n", gps_data.time);
	  printf("Latitude: %s %c\n", gps_data.latitude, gps_data.latitude_indicator);
	  printf("Longitude: %s %c\n", gps_data.longitude, gps_data.longitude_indicator);
	  printf("Vitesse: %s\n", gps_data.speed_knots);
	  printf("Angle: %s\n", gps_data.track_angle);
	}
	*/
	// End test
}

uint8_t STM32_GetStates(uint8_t mode) {

	states.mode = mode & 0x03;
	states.pyro1 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_0, VREF12) & 0x01;
	states.pyro2 = CD74HC4051_AnRead(&hadc1, CHANNEL_0, PYRO_CHANNEL_1, VREF12) & 0x01;
	if(mode == MODE_PREFLIGHT) {

		// Faire fonctions checkvalidity...

		states.accelerometer = 0x00 & 0x01;
		states.barometer = 0x00 & 0x01;
		states.gps = 0x00 & 0x01;
		states.sd = MEM2067_SDCardDetection() & 0x01;
		return (states.mode << 6) | (states.pyro1 << 5) | (states.pyro2 << 4) | (states.accelerometer << 3) | (states.barometer << 2) | (states.gps << 1) | states.sd;
	} else if(mode == MODE_FLIGHT || MODE_POSTFLIGHT) {
		return (states.mode << 6) | (states.pyro1 << 5) | (states.pyro2 << 4) | 0x00;
	} else {
		return 0x01; // Error
	}
}

uint8_t STM32_ModeRoutine(uint8_t mode) {
	uint8_t *data;
	if(mode == MODE_PREFLIGHT) {
		// V_Batt_Lipo1(16);V_Batt_Lipo2(16);V_Batt_Lipo3(16);5V_AN(16);Temp(32);Altitude(16);AngleRoll(32);AnglePitch(32) => (22)
		data = (bool *)malloc(22 * sizeof(bool));
		if (data == NULL) {
			return 0;
		}

	} else if (mode == MODE_FLIGHT) {
		// GPS_Data(416);Gyro_Data(72) => (64)
		data = (bool *)malloc(FLIGHT_SIZE * sizeof(bool));
		if (data == NULL) {
			return 0;
		}

	} else if(mode == MODE_POSTFLIGHT) {
		// data: gps;altitude;vbat
		data = (bool *)malloc(POSTFLIGHT_SIZE * sizeof(bool));
		if (data == NULL) {
			return 0;
		}

	} else {
		return 0;
	}
	return 1; // OK
}

int main(void)
{
  /* USER CODE BEGIN 1 */
	STM32_InitRoutine();
  /* USER CODE END 1 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

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
