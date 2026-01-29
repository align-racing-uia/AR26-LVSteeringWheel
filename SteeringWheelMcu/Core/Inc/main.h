/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define Rotary14_Pin GPIO_PIN_12
#define Rotary14_GPIO_Port GPIOB
#define Rotary13_Pin GPIO_PIN_13
#define Rotary13_GPIO_Port GPIOB
#define Rotary12_Pin GPIO_PIN_14
#define Rotary12_GPIO_Port GPIOB
#define Rotary11_Pin GPIO_PIN_15
#define Rotary11_GPIO_Port GPIOB
#define Button4_Pin GPIO_PIN_6
#define Button4_GPIO_Port GPIOC
#define Button3_Pin GPIO_PIN_7
#define Button3_GPIO_Port GPIOC
#define Button2_Pin GPIO_PIN_8
#define Button2_GPIO_Port GPIOC
#define Button1_Pin GPIO_PIN_9
#define Button1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
