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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define check_Pin GPIO_PIN_13
#define check_GPIO_Port GPIOC
#define VRY_Pin GPIO_PIN_0
#define VRY_GPIO_Port GPIOA
#define VRX_Pin GPIO_PIN_1
#define VRX_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_2
#define IRQ_GPIO_Port GPIOA
#define IRQ_EXTI_IRQn EXTI2_IRQn
#define CSN_Pin GPIO_PIN_3
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_4
#define CE_GPIO_Port GPIOA
#define SW_Pin GPIO_PIN_0
#define SW_GPIO_Port GPIOB
#define RGB_1_Pin GPIO_PIN_1
#define RGB_1_GPIO_Port GPIOB
#define RGB_2_Pin GPIO_PIN_2
#define RGB_2_GPIO_Port GPIOB
#define RGB_3_Pin GPIO_PIN_10
#define RGB_3_GPIO_Port GPIOB
#define Led_Pin GPIO_PIN_11
#define Led_GPIO_Port GPIOB
#define B_8_Pin GPIO_PIN_11
#define B_8_GPIO_Port GPIOA
#define B_7_Pin GPIO_PIN_12
#define B_7_GPIO_Port GPIOA
#define B_6_Pin GPIO_PIN_15
#define B_6_GPIO_Port GPIOA
#define B_5_Pin GPIO_PIN_3
#define B_5_GPIO_Port GPIOB
#define B_4_Pin GPIO_PIN_4
#define B_4_GPIO_Port GPIOB
#define B_3_Pin GPIO_PIN_5
#define B_3_GPIO_Port GPIOB
#define B_2_Pin GPIO_PIN_8
#define B_2_GPIO_Port GPIOB
#define B_1_Pin GPIO_PIN_9
#define B_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
