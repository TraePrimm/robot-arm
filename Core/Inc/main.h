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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

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
#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC
#define Joint_2_EncoderCS_Pin GPIO_PIN_2
#define Joint_2_EncoderCS_GPIO_Port GPIOC
#define Joint_2_Dir_Pin GPIO_PIN_3
#define Joint_2_Dir_GPIO_Port GPIOC
#define Joint_5_Step_Pin GPIO_PIN_0
#define Joint_5_Step_GPIO_Port GPIOA
#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA
#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA
#define Joint_0_EncoderCS_Pin GPIO_PIN_4
#define Joint_0_EncoderCS_GPIO_Port GPIOA
#define Joint_0_Step_Pin GPIO_PIN_5
#define Joint_0_Step_GPIO_Port GPIOA
#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC
#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC
#define Joint_1_EncoderCS_Pin GPIO_PIN_2
#define Joint_1_EncoderCS_GPIO_Port GPIOB
#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define Joint_4_Step_Pin GPIO_PIN_15
#define Joint_4_Step_GPIO_Port GPIOB
#define Joint_2_Step_Pin GPIO_PIN_12
#define Joint_2_Step_GPIO_Port GPIOD
#define Joint_3_Step_Pin GPIO_PIN_6
#define Joint_3_Step_GPIO_Port GPIOC
#define Joint_1_Step_Pin GPIO_PIN_8
#define Joint_1_Step_GPIO_Port GPIOC
#define Joint_1_DIR_Pin GPIO_PIN_9
#define Joint_1_DIR_GPIO_Port GPIOC
#define Joint_3_EncoderCS_Pin GPIO_PIN_10
#define Joint_3_EncoderCS_GPIO_Port GPIOC
#define Joint_3_Dir_Pin GPIO_PIN_11
#define Joint_3_Dir_GPIO_Port GPIOC
#define Joint_4_EncoderCS_Pin GPIO_PIN_0
#define Joint_4_EncoderCS_GPIO_Port GPIOD
#define Joint_4_Dir_Pin GPIO_PIN_1
#define Joint_4_Dir_GPIO_Port GPIOD
#define Joint_5_EncoderCS_Pin GPIO_PIN_2
#define Joint_5_EncoderCS_GPIO_Port GPIOD
#define Joint_5_Dir_Pin GPIO_PIN_3
#define Joint_5_Dir_GPIO_Port GPIOD
#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOG
#define RMII_REF_CLKG13_Pin GPIO_PIN_13
#define RMII_REF_CLKG13_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
