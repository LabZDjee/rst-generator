/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

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
// MAX PERIOD IS 1,600 (check in .ioc if this hasn't been changed)
#define PWM_MIN_COUNT (1*160) // 10 % for maximum angle
#define PWM_MAX_COUNT (8*160) // 80 % for minimum angle
// Ensures PWM duty cycle follows PWM selector state
#define REFRESH_PWM_DC \
		htim2.Instance->CCR1 = HAL_GPIO_ReadPin(PWM_selector_GPIO_Port, PWM_selector_Pin) == GPIO_PIN_RESET ? PWM_MAX_COUNT : PWM_MIN_COUNT;
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_out_Pin GPIO_PIN_0
#define PWM_out_GPIO_Port GPIOA
#define T_relay_Pin GPIO_PIN_1
#define T_relay_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define S_relay_Pin GPIO_PIN_3
#define S_relay_GPIO_Port GPIOA
#define R_relay_Pin GPIO_PIN_4
#define R_relay_GPIO_Port GPIOA
#define T_push_pull_Pin GPIO_PIN_5
#define T_push_pull_GPIO_Port GPIOA
#define S_push_pull_Pin GPIO_PIN_6
#define S_push_pull_GPIO_Port GPIOA
#define R_push_pull_Pin GPIO_PIN_7
#define R_push_pull_GPIO_Port GPIOA
#define PWM_selector_Pin GPIO_PIN_0
#define PWM_selector_GPIO_Port GPIOB
#define PWM_selector_EXTI_IRQn EXTI0_1_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define Reversed_Ph_Pin GPIO_PIN_4
#define Reversed_Ph_GPIO_Port GPIOB
#define R_off_Pin GPIO_PIN_5
#define R_off_GPIO_Port GPIOB
#define S_off_Pin GPIO_PIN_6
#define S_off_GPIO_Port GPIOB
#define T_off_Pin GPIO_PIN_7
#define T_off_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
