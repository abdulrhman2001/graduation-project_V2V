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
#define Trigger0_Pin GPIO_PIN_0
#define Trigger0_GPIO_Port GPIOC
#define Echo0_Pin GPIO_PIN_1
#define Echo0_GPIO_Port GPIOC
#define Trigger1_Pin GPIO_PIN_2
#define Trigger1_GPIO_Port GPIOC
#define Echo1_Pin GPIO_PIN_3
#define Echo1_GPIO_Port GPIOC
#define GREEN_LED_Pin GPIO_PIN_5
#define GREEN_LED_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_4
#define ENABLE_GPIO_Port GPIOC
#define IN4_Pin GPIO_PIN_1
#define IN4_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_2
#define RS_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_13
#define IN3_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_14
#define IN2_GPIO_Port GPIOB
#define IN1_Pin GPIO_PIN_15
#define IN1_GPIO_Port GPIOB
#define Trigger2_Pin GPIO_PIN_10
#define Trigger2_GPIO_Port GPIOC
#define Echo2_Pin GPIO_PIN_12
#define Echo2_GPIO_Port GPIOC
#define D4_Pin GPIO_PIN_4
#define D4_GPIO_Port GPIOB
#define D5_Pin GPIO_PIN_5
#define D5_GPIO_Port GPIOB
#define D6_Pin GPIO_PIN_6
#define D6_GPIO_Port GPIOB
#define D7_Pin GPIO_PIN_7
#define D7_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
