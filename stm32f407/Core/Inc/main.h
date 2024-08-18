/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define Sense1_Pin GPIO_PIN_0
#define Sense1_GPIO_Port GPIOA
#define Sense2_Pin GPIO_PIN_1
#define Sense2_GPIO_Port GPIOA
#define Sense3_Pin GPIO_PIN_2
#define Sense3_GPIO_Port GPIOA
#define In1_1_Pin GPIO_PIN_10
#define In1_1_GPIO_Port GPIOE
#define In2_1_Pin GPIO_PIN_12
#define In2_1_GPIO_Port GPIOE
#define In3_1_Pin GPIO_PIN_14
#define In3_1_GPIO_Port GPIOE
#define En3_Pin GPIO_PIN_10
#define En3_GPIO_Port GPIOB
#define En1_Pin GPIO_PIN_15
#define En1_GPIO_Port GPIOA
#define En2_Pin GPIO_PIN_3
#define En2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
