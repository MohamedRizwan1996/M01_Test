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
#include "gpio.h"
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
#define Led_Pin GPIO_PIN_13
#define Led_GPIO_Port GPIOC
#define Led3_Pin GPIO_PIN_2
#define Led3_GPIO_Port GPIOC
#define Led2_Pin GPIO_PIN_3
#define Led2_GPIO_Port GPIOC
#define IN_LS_1_Pin GPIO_PIN_2
#define IN_LS_1_GPIO_Port GPIOB
#define IN_LS_1_EXTI_IRQn EXTI2_IRQn
#define IN_LS_2_Pin GPIO_PIN_10
#define IN_LS_2_GPIO_Port GPIOB
#define IN_LS_2_EXTI_IRQn EXTI15_10_IRQn
#define IN_LS_3_Pin GPIO_PIN_12
#define IN_LS_3_GPIO_Port GPIOB
#define IN_LS_3_EXTI_IRQn EXTI15_10_IRQn
#define IN_LS_4_Pin GPIO_PIN_13
#define IN_LS_4_GPIO_Port GPIOB
#define IN_LS_4_EXTI_IRQn EXTI15_10_IRQn
#define IN_LS_5_Pin GPIO_PIN_14
#define IN_LS_5_GPIO_Port GPIOB
#define IN_LS_5_EXTI_IRQn EXTI15_10_IRQn
#define IN_LS_6_Pin GPIO_PIN_15
#define IN_LS_6_GPIO_Port GPIOB
#define IN_LS_6_EXTI_IRQn EXTI15_10_IRQn
#define IN_LS_7_Pin GPIO_PIN_6
#define IN_LS_7_GPIO_Port GPIOC
#define IN_LS_7_EXTI_IRQn EXTI9_5_IRQn
#define IN_LS_8_Pin GPIO_PIN_7
#define IN_LS_8_GPIO_Port GPIOC
#define IN_LS_8_EXTI_IRQn EXTI9_5_IRQn
#define IN_LS_9_Pin GPIO_PIN_8
#define IN_LS_9_GPIO_Port GPIOC
#define IN_LS_9_EXTI_IRQn EXTI9_5_IRQn
#define IN_LS_10_Pin GPIO_PIN_11
#define IN_LS_10_GPIO_Port GPIOA
#define IN_LS_10_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

#define SYSTEM_M01_BRAKING_STEERING_TEST	1
#define SYSTEM_M01_LIFTER_TEST	2

#define SYSTEM_M01	SYSTEM_M01_BRAKING_STEERING_TEST
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
