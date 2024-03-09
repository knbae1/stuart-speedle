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
#define RECIV_FR_Pin GPIO_PIN_4
#define RECIV_FR_GPIO_Port GPIOA
#define RECIV_R_Pin GPIO_PIN_5
#define RECIV_R_GPIO_Port GPIOA
#define RECIV_L_Pin GPIO_PIN_0
#define RECIV_L_GPIO_Port GPIOB
#define RECIV_FL_Pin GPIO_PIN_1
#define RECIV_FL_GPIO_Port GPIOB
#define EMIT_FL_Pin GPIO_PIN_12
#define EMIT_FL_GPIO_Port GPIOB
#define MR_FWD_Pin GPIO_PIN_13
#define MR_FWD_GPIO_Port GPIOB
#define ML_BWD_Pin GPIO_PIN_14
#define ML_BWD_GPIO_Port GPIOB
#define MR_BWD_Pin GPIO_PIN_15
#define MR_BWD_GPIO_Port GPIOB
#define ML_FWD_Pin GPIO_PIN_8
#define ML_FWD_GPIO_Port GPIOA
#define EMIT_FR_Pin GPIO_PIN_5
#define EMIT_FR_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */