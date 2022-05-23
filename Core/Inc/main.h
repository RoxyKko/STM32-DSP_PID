/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define KEY_col3_Pin GPIO_PIN_2
#define KEY_col3_GPIO_Port GPIOE
#define KEY_col2_Pin GPIO_PIN_4
#define KEY_col2_GPIO_Port GPIOE
#define KEY_col1_Pin GPIO_PIN_6
#define KEY_col1_GPIO_Port GPIOE
#define KEY_col0_Pin GPIO_PIN_0
#define KEY_col0_GPIO_Port GPIOF
#define KEY_row3_Pin GPIO_PIN_2
#define KEY_row3_GPIO_Port GPIOF
#define KEY_row2_Pin GPIO_PIN_4
#define KEY_row2_GPIO_Port GPIOF
#define KEY_row0_Pin GPIO_PIN_5
#define KEY_row0_GPIO_Port GPIOF
#define KEY_row1_Pin GPIO_PIN_6
#define KEY_row1_GPIO_Port GPIOF
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOF
#define Lock_Pin GPIO_PIN_4
#define Lock_GPIO_Port GPIOA
#define ROM_IN_Pin GPIO_PIN_4
#define ROM_IN_GPIO_Port GPIOD
#define ROM_OUT_Pin GPIO_PIN_6
#define ROM_OUT_GPIO_Port GPIOD
#define ROM_SCK_Pin GPIO_PIN_9
#define ROM_SCK_GPIO_Port GPIOG
#define ROM_CS_Pin GPIO_PIN_11
#define ROM_CS_GPIO_Port GPIOG
#define LED0_Pin GPIO_PIN_13
#define LED0_GPIO_Port GPIOG
#define SCLK_Pin GPIO_PIN_15
#define SCLK_GPIO_Port GPIOG
#define SDA_Pin GPIO_PIN_4
#define SDA_GPIO_Port GPIOB
#define RS_Pin GPIO_PIN_6
#define RS_GPIO_Port GPIOB
#define REST_Pin GPIO_PIN_8
#define REST_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_0
#define CS_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
