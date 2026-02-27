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

void usb_cdc_rx_callback(uint8_t* buf, size_t size);
void usb_cdc_tx_callback(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_DB_Pin GPIO_PIN_13
#define LED_DB_GPIO_Port GPIOC
#define EXTI0_HALL_Pin GPIO_PIN_0
#define EXTI0_HALL_GPIO_Port GPIOA
#define EXTI0_HALL_EXTI_IRQn EXTI0_IRQn
#define BTN_MB_Pin GPIO_PIN_4
#define BTN_MB_GPIO_Port GPIOA
#define HALL_EN_Pin GPIO_PIN_7
#define HALL_EN_GPIO_Port GPIOA
#define AHT20_EN_Pin GPIO_PIN_1
#define AHT20_EN_GPIO_Port GPIOB
#define SENS_EN_Pin GPIO_PIN_2
#define SENS_EN_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define CHRG_OK_Pin GPIO_PIN_8
#define CHRG_OK_GPIO_Port GPIOA
#define CHRG_STAT_Pin GPIO_PIN_9
#define CHRG_STAT_GPIO_Port GPIOA
#define MDM_EN_Pin GPIO_PIN_10
#define MDM_EN_GPIO_Port GPIOA
#define EXT_WDG_Pin GPIO_PIN_15
#define EXT_WDG_GPIO_Port GPIOA
#define VBAT_MEAS_EN_Pin GPIO_PIN_5
#define VBAT_MEAS_EN_GPIO_Port GPIOB
#define MDM_EN_PRE_Pin GPIO_PIN_4
#define MDM_EN_PRE_GPIO_Port GPIOB
#define LED_MB_Pin GPIO_PIN_8
#define LED_MB_GPIO_Port GPIOB
#define MDM_RST_Pin GPIO_PIN_9
#define MDM_RST_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
