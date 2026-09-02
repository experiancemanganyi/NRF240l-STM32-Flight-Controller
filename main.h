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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define CURRENT_Pin GPIO_PIN_0
#define CURRENT_GPIO_Port GPIOA
#define VOLTAGE_Pin GPIO_PIN_1
#define VOLTAGE_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_2
#define NRF_CE_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_3
#define NRF_CSN_GPIO_Port GPIOA
#define ESC_Pin GPIO_PIN_0
#define ESC_GPIO_Port GPIOB
#define INTERUPT_MPU_Pin GPIO_PIN_1
#define INTERUPT_MPU_GPIO_Port GPIOB
#define INTERUPT_MPU_EXTI_IRQn EXTI1_IRQn
#define INTERUPT_NRF_Pin GPIO_PIN_10
#define INTERUPT_NRF_GPIO_Port GPIOB
#define INTERUPT_NRF_EXTI_IRQn EXTI15_10_IRQn
#define SERVO_1_Pin GPIO_PIN_8
#define SERVO_1_GPIO_Port GPIOA
#define SERVO_2_Pin GPIO_PIN_9
#define SERVO_2_GPIO_Port GPIOA
#define SERVO_3_Pin GPIO_PIN_10
#define SERVO_3_GPIO_Port GPIOA
#define SERVO_4_Pin GPIO_PIN_4
#define SERVO_4_GPIO_Port GPIOB
#define LED_Lights_Pin GPIO_PIN_5
#define LED_Lights_GPIO_Port GPIOB
#define SERVO_5_Pin GPIO_PIN_8
#define SERVO_5_GPIO_Port GPIOB
#define SERVO_6_Pin GPIO_PIN_9
#define SERVO_6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
