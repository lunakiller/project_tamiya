/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

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
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define THROTTLE_Pin GPIO_PIN_0
#define THROTTLE_GPIO_Port GPIOA
#define STEERING_Pin GPIO_PIN_1
#define STEERING_GPIO_Port GPIOA
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOA
#define LED_B_Pin GPIO_PIN_0
#define LED_B_GPIO_Port GPIOB
#define BAT_Pin GPIO_PIN_1
#define BAT_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_10
#define SW1_GPIO_Port GPIOB
#define SW1_EXTI_IRQn EXTI15_10_IRQn
#define SW2_Pin GPIO_PIN_11
#define SW2_GPIO_Port GPIOB
#define SW2_EXTI_IRQn EXTI15_10_IRQn
#define NRF_IRQ_Pin GPIO_PIN_12
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_IRQ_EXTI_IRQn EXTI15_10_IRQn
#define NRF_SCK_Pin GPIO_PIN_13
#define NRF_SCK_GPIO_Port GPIOB
#define NRF_MISO_Pin GPIO_PIN_14
#define NRF_MISO_GPIO_Port GPIOB
#define NRF_MOSI_Pin GPIO_PIN_15
#define NRF_MOSI_GPIO_Port GPIOB
#define NRF_CSN_Pin GPIO_PIN_8
#define NRF_CSN_GPIO_Port GPIOA
#define NRF_CE_Pin GPIO_PIN_9
#define NRF_CE_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define ENC1_CH1_Pin GPIO_PIN_15
#define ENC1_CH1_GPIO_Port GPIOA
#define ENC1_CH2_Pin GPIO_PIN_3
#define ENC1_CH2_GPIO_Port GPIOB
#define ENC1_BTN_Pin GPIO_PIN_4
#define ENC1_BTN_GPIO_Port GPIOB
#define ENC1_BTN_EXTI_IRQn EXTI4_IRQn
#define ENC2_BTN_Pin GPIO_PIN_5
#define ENC2_BTN_GPIO_Port GPIOB
#define ENC2_BTN_EXTI_IRQn EXTI9_5_IRQn
#define ENC2_CH1_Pin GPIO_PIN_6
#define ENC2_CH1_GPIO_Port GPIOB
#define ENC2_CH2_Pin GPIO_PIN_7
#define ENC2_CH2_GPIO_Port GPIOB
#define OLED_SCL_Pin GPIO_PIN_8
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_9
#define OLED_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
