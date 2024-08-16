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
#include "util.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct pixel channel_framebuffers[WS2812_NUM_CHANNELS][FRAMEBUFFER_SIZE];
struct led_channel_info led_channels[WS2812_NUM_CHANNELS];

// Constructor
RunTimer run_timer;
BMP280 bmp_data;
ICM20602 icm_data;
L76LM33 L76_data;
RFD900 rfd_data;
HM10BLE ble_data;
ROCKET_Data rocket_data;
// Buffers
uint8_t HM10BLE_buffer[20];  // ble

// Variables
char* filename_log = "log.txt";
uint8_t rocket_behavior = 0x00;
bool pyro_armed = false;
bool push_button = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
static void MX_SPI1_Init(void);
void TIM3_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  MX_SPI1_Init();
  TIM3_Init();
  ROCKET_InitRoutine();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	rocket_behavior = ROCKET_Behavior();
	ROCKET_ModeRoutine();

	/*
	if(HM10BLE_ConnectionStatus(&ble_data)) {
		printf("Connected\r\n");
	} else printf("Not connected\r\n");
	*/

    if((rocket_behavior & ACCZ_MASK) != 0) {
    	if(bmp_data.altitude_filtered_m >= ALTITUDE_START) {
    		ROCKET_SetMode(MODE_INFLIGHT);
    		pyro_armed = true;
    	}
    } else if((rocket_behavior & ACCZ_MASK) == 0 && rocket_data.header_states.pyro0 == 0 && rocket_data.header_states.pyro1 == 0) {
    	ROCKET_SetMode(MODE_POSTFLIGHT);
    } else {
		ROCKET_SetMode(MODE_PREFLIGHT);
		pyro_armed = false;
	}
    // Mach Lock
    if((rocket_behavior & MACHLOCK_MASK) == 0) {
    	// Altitude
		if((rocket_behavior & ALTITUDE_MASK) != 0) {
			Pyro_Fire(pyro_armed, 0);
			// TODO: add altitude + time mem2067
			MEM2067_Write(FILENAME_LOG, "Time: ... / Altitude: ... -> Pyro1 release\r\n");
			if(bmp_data.altitude_filtered_m <= ALTITUDE_PYRO2) {
				Pyro_Fire(pyro_armed, 1);
				// TODO: add altitude + time mem2067
				MEM2067_Write(FILENAME_LOG, "Time: ... / Altitude: ... -> Pyro2 release\r\n");
			}
		}
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_2)
  {
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {

  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_2, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(72000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
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

/* USER CODE BEGIN 4 */
static void MX_SPI1_Init(void)
{
    /* Activer l'horloge pour le périphérique SPI1 */
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // Activer l'horloge SPI1

    /* Configurer les broches pour SPI1 (MOSI, MISO, SCK) */
    // Remplacez les définitions des broches par celles que vous utilisez
    GPIOB->CRH &= ~(GPIO_CRH_MODE12 | GPIO_CRH_CNF12); // SCK (PB12)
    GPIOB->CRH |= (GPIO_CRH_MODE12_1 | GPIO_CRH_CNF12_1); // PB12: Output, Max speed 2 MHz, Push-pull

    GPIOB->CRH &= ~(GPIO_CRH_MODE15 | GPIO_CRH_CNF15); // MOSI (PB15)
    GPIOB->CRH |= (GPIO_CRH_MODE15_1 | GPIO_CRH_CNF15_1); // PB15: Output, Max speed 2 MHz, Push-pull

    GPIOB->CRH &= ~(GPIO_CRH_MODE14 | GPIO_CRH_CNF14); // MISO (PB14)
    GPIOB->CRH |= (GPIO_CRH_CNF14_0); // PB14: Input floating

    /* Configurer le périphérique SPI1 */
    SPI1->CR1 = 0; // Réinitialiser CR1
    SPI1->CR1 |= (SPI_CR1_MSTR |                // Mode maître
                  SPI_CR1_BR_1 |                 // Baud rate prescaler: f_PCLK/32
                  SPI_CR1_CPOL |                 // CLK Polarity Low
                  SPI_CR1_CPHA |                 // CLK Phase 1 Edge
                  SPI_CR1_LSBFIRST |             // MSB first
                  SPI_CR1_SSM |                  // Software slave management
                  SPI_CR1_SSI |                  // Internal slave select
                  SPI_CR1_RXONLY);               // Réception uniquement

    // Activer le SPI1
    SPI1->CR1 |= SPI_CR1_SPE; // Activer SPI1
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  L76LM33_RxCallback(&L76_data, huart);
}

void TIM3_Init(void) {
    // Activer l'horloge pour TIM3
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    // Configurer le Prescaler et la Période
    TIM3->PSC = 127;
    TIM3->ARR = 20;

    // Configurer le Mode de Compteur
    TIM3->CR1 &= ~TIM_CR1_DIR;    // Compteur ascendant
    TIM3->CR1 &= ~TIM_CR1_CMS;    // Mode de compteur aligné sur le bord
    TIM3->CR1 |= TIM_CR1_ARPE;    // Auto-reload preload enable

    // Configurer le Mode PWM pour le canal 4
    TIM3->CCMR2 &= ~TIM_CCMR2_OC4M;       // Clear output compare mode bits
    TIM3->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos); // PWM mode 1 (OC4M bits = 110)
    TIM3->CCMR2 &= ~TIM_CCMR2_OC4PE;      // Disable preload (OC4PE bit)
    TIM3->CCER |= TIM_CCER_CC4E;          // Enable output for channel 4
    TIM3->CCR4 = 0;                       // Initial pulse width

    // Démarrer le Timer
    TIM3->CR1 |= TIM_CR1_CEN;
}

void EXTI9_5_IRQHandler(void)
{
    // Vérifier si l'interruption provient de la ligne 9
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_9))
    {
        printf("button\r\n");

        // Effacer le drapeau d'interruption
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_9);
    }
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
    while (1) {
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
     ex: prinf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
