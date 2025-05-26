/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32wbxx_hal.h"

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
#define EN2_Pin GPIO_PIN_2
#define EN2_GPIO_Port GPIOA
#define BAT_SENSE_Pin GPIO_PIN_1
#define BAT_SENSE_GPIO_Port GPIOA
#define IMU_INT1_Pin GPIO_PIN_0
#define IMU_INT1_GPIO_Port GPIOA
#define LED_5V_EN_Pin GPIO_PIN_2
#define LED_5V_EN_GPIO_Port GPIOC
#define BUTTON5_Pin GPIO_PIN_3
#define BUTTON5_GPIO_Port GPIOH
#define BUTTON5_EXTI_IRQn EXTI3_IRQn
#define BUTTON1_Pin GPIO_PIN_7
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON1_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON2_Pin GPIO_PIN_5
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON2_EXTI_IRQn EXTI9_5_IRQn
#define BUTTON3_Pin GPIO_PIN_4
#define BUTTON3_GPIO_Port GPIOB
#define BUTTON3_EXTI_IRQn EXTI4_IRQn
#define BUTTON4_Pin GPIO_PIN_10
#define BUTTON4_GPIO_Port GPIOC
#define BUTTON4_EXTI_IRQn EXTI15_10_IRQn
#define MUX_RESET_Pin GPIO_PIN_15
#define MUX_RESET_GPIO_Port GPIOA
#define nFAULT13_Pin GPIO_PIN_0
#define nFAULT13_GPIO_Port GPIOD
#define nFAULT17_Pin GPIO_PIN_1
#define nFAULT17_GPIO_Port GPIOD
#define nFAULT15_Pin GPIO_PIN_13
#define nFAULT15_GPIO_Port GPIOB
#define nFAULT8_Pin GPIO_PIN_12
#define nFAULT8_GPIO_Port GPIOB
#define nFAULT9_Pin GPIO_PIN_4
#define nFAULT9_GPIO_Port GPIOE
#define nFAULT12_Pin GPIO_PIN_4
#define nFAULT12_GPIO_Port GPIOC
#define nFAULT11_Pin GPIO_PIN_8
#define nFAULT11_GPIO_Port GPIOA
#define nFAULT16_Pin GPIO_PIN_7
#define nFAULT16_GPIO_Port GPIOA
#define nFAULT0_Pin GPIO_PIN_6
#define nFAULT0_GPIO_Port GPIOA
#define nFAULT10_Pin GPIO_PIN_5
#define nFAULT10_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_3
#define EN1_GPIO_Port GPIOA
#define nFAULT2_Pin GPIO_PIN_0
#define nFAULT2_GPIO_Port GPIOH
#define BAT_SENSE_EN_Pin GPIO_PIN_1
#define BAT_SENSE_EN_GPIO_Port GPIOH
#define nFAULT3_Pin GPIO_PIN_14
#define nFAULT3_GPIO_Port GPIOD
#define nSLEEP_FRONT_Pin GPIO_PIN_1
#define nSLEEP_FRONT_GPIO_Port GPIOE
#define nFAULT4_Pin GPIO_PIN_13
#define nFAULT4_GPIO_Port GPIOD
#define IMU_INT2_Pin GPIO_PIN_12
#define IMU_INT2_GPIO_Port GPIOD
#define nFAULT14_Pin GPIO_PIN_3
#define nFAULT14_GPIO_Port GPIOD
#define nSLEEP_REAR_Pin GPIO_PIN_7
#define nSLEEP_REAR_GPIO_Port GPIOC
#define SWITCH1_Pin GPIO_PIN_3
#define SWITCH1_GPIO_Port GPIOE
#define SWITCH2_Pin GPIO_PIN_4
#define SWITCH2_GPIO_Port GPIOD
#define CHARGE_STATUS_Pin GPIO_PIN_15
#define CHARGE_STATUS_GPIO_Port GPIOD
#define nFAULT1_Pin GPIO_PIN_10
#define nFAULT1_GPIO_Port GPIOD
#define nFAULT7_Pin GPIO_PIN_2
#define nFAULT7_GPIO_Port GPIOE
#define nFAULT6_Pin GPIO_PIN_0
#define nFAULT6_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
