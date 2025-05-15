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
#define check_led_Pin GPIO_PIN_13
#define check_led_GPIO_Port GPIOC
#define sensor_distance_Pin GPIO_PIN_1
#define sensor_distance_GPIO_Port GPIOA
#define IRQ_Pin GPIO_PIN_2
#define IRQ_GPIO_Port GPIOA
#define IRQ_EXTI_IRQn EXTI2_IRQn
#define CSN_Pin GPIO_PIN_3
#define CSN_GPIO_Port GPIOA
#define CE_Pin GPIO_PIN_4
#define CE_GPIO_Port GPIOA
#define EN_M_Left_Pin GPIO_PIN_0
#define EN_M_Left_GPIO_Port GPIOB
#define EN_M_Right_Pin GPIO_PIN_1
#define EN_M_Right_GPIO_Port GPIOB
#define OLED_DC_Pin GPIO_PIN_2
#define OLED_DC_GPIO_Port GPIOB
#define L298_M_IN2_Pin GPIO_PIN_10
#define L298_M_IN2_GPIO_Port GPIOB
#define OLED_RST_Pin GPIO_PIN_11
#define OLED_RST_GPIO_Port GPIOB
#define L298_M_IN1_Pin GPIO_PIN_12
#define L298_M_IN1_GPIO_Port GPIOB
#define OLED_CS_Pin GPIO_PIN_14
#define OLED_CS_GPIO_Port GPIOB
#define L298_M_IN3_Pin GPIO_PIN_8
#define L298_M_IN3_GPIO_Port GPIOA
#define L298_M_IN4_Pin GPIO_PIN_11
#define L298_M_IN4_GPIO_Port GPIOA
#define sensor_Left_Pin GPIO_PIN_12
#define sensor_Left_GPIO_Port GPIOA
#define sensor_Straight_Pin GPIO_PIN_15
#define sensor_Straight_GPIO_Port GPIOA
#define sensor_Right_Pin GPIO_PIN_3
#define sensor_Right_GPIO_Port GPIOB
#define EN_Left_Pin GPIO_PIN_4
#define EN_Left_GPIO_Port GPIOB
#define EN_Right_Pin GPIO_PIN_5
#define EN_Right_GPIO_Port GPIOB
#define Buzzer_Pin GPIO_PIN_6
#define Buzzer_GPIO_Port GPIOB
#define servo_1_Pin GPIO_PIN_7
#define servo_1_GPIO_Port GPIOB
#define servo_2_Pin GPIO_PIN_8
#define servo_2_GPIO_Port GPIOB
#define sensor_Back_Pin GPIO_PIN_9
#define sensor_Back_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
