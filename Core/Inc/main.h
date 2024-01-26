/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MUL_S0_Pin GPIO_PIN_13
#define MUL_S0_GPIO_Port GPIOC
#define MUL_S1_Pin GPIO_PIN_14
#define MUL_S1_GPIO_Port GPIOC
#define MUL_S2_Pin GPIO_PIN_15
#define MUL_S2_GPIO_Port GPIOC
#define BATT3S2_ON_Pin GPIO_PIN_1
#define BATT3S2_ON_GPIO_Port GPIOA
#define GPS_TX_Pin GPIO_PIN_2
#define GPS_TX_GPIO_Port GPIOA
#define GPS_RX_Pin GPIO_PIN_3
#define GPS_RX_GPIO_Port GPIOA
#define SDC_CS_Pin GPIO_PIN_4
#define SDC_CS_GPIO_Port GPIOA
#define CAM_ON_Pin GPIO_PIN_0
#define CAM_ON_GPIO_Port GPIOB
#define BUZZER_ON_Pin GPIO_PIN_1
#define BUZZER_ON_GPIO_Port GPIOB
#define LED_STATUS_Pin GPIO_PIN_2
#define LED_STATUS_GPIO_Port GPIOB
#define BT_TX_Pin GPIO_PIN_10
#define BT_TX_GPIO_Port GPIOB
#define BT_RX_Pin GPIO_PIN_11
#define BT_RX_GPIO_Port GPIOB
#define ICM_CS_Pin GPIO_PIN_12
#define ICM_CS_GPIO_Port GPIOB
#define BMP_CS_Pin GPIO_PIN_8
#define BMP_CS_GPIO_Port GPIOA
#define BUTTON_IN_Pin GPIO_PIN_9
#define BUTTON_IN_GPIO_Port GPIOA
#define ICM_INT_Pin GPIO_PIN_10
#define ICM_INT_GPIO_Port GPIOA
#define NPYRO_TEST_Pin GPIO_PIN_15
#define NPYRO_TEST_GPIO_Port GPIOA
#define PYRO_ON0_Pin GPIO_PIN_4
#define PYRO_ON0_GPIO_Port GPIOB
#define PYRO_ON1_Pin GPIO_PIN_5
#define PYRO_ON1_GPIO_Port GPIOB
#define RFD_TX_Pin GPIO_PIN_6
#define RFD_TX_GPIO_Port GPIOB
#define RFD_RX_Pin GPIO_PIN_7
#define RFD_RX_GPIO_Port GPIOB
#define NMUL_E_Pin GPIO_PIN_8
#define NMUL_E_GPIO_Port GPIOB
#define CHECK12V_Pin GPIO_PIN_9
#define CHECK12V_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
