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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOD
#define highSpeedPin1_Pin GPIO_PIN_10
#define highSpeedPin1_GPIO_Port GPIOC
#define reversePin1_Pin GPIO_PIN_11
#define reversePin1_GPIO_Port GPIOC
#define lowSpeedPin1_Pin GPIO_PIN_12
#define lowSpeedPin1_GPIO_Port GPIOC
#define cruisePin1_Pin GPIO_PIN_0
#define cruisePin1_GPIO_Port GPIOD
#define highSpeedPin2_Pin GPIO_PIN_1
#define highSpeedPin2_GPIO_Port GPIOD
#define reversePin2_Pin GPIO_PIN_2
#define reversePin2_GPIO_Port GPIOD
#define lowSpeedPin2_Pin GPIO_PIN_3
#define lowSpeedPin2_GPIO_Port GPIOD
#define cruisePin2_Pin GPIO_PIN_4
#define cruisePin2_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
