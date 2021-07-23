/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32l0xx_it.c
 * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
#define NO (0)
#define YES (!NO)

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
uint8_t tsr[2][6][3] = { {
    { GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET }, //   0 -  60
    { GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET }, //  60 - 120
    { GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET },   // 120 - 180
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET }, // 180 - 240
    { GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET },   // 240 - 300
    { GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET } }, { // 300 - 360
    { GPIO_PIN_SET, GPIO_PIN_SET, GPIO_PIN_RESET },  //   0 -  60
    { GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_RESET },  //  60 - 120
    { GPIO_PIN_RESET, GPIO_PIN_SET, GPIO_PIN_SET },  // 120 - 180
    { GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_SET },  // 180 - 240
    { GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_SET },  // 240 - 300
    { GPIO_PIN_SET, GPIO_PIN_RESET, GPIO_PIN_RESET } } }; // 300 - 360
uint8_t tsrCanChangeDirection[6] = { YES, NO, NO, YES, NO, NO };

static uint8_t tsrDirection = 0; // 0: normal, 1: reversed
static uint8_t tsrIndex = 0; // from 0 to 5: second index of tsr LUT

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim21;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim2;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1) {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and line 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */
  if(__HAL_GPIO_EXTI_GET_FLAG(PWM_selector_Pin)){
    REFRESH_PWM_DC
  }
  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
  * @brief This function handles TIM21 global interrupt.
  */
void TIM21_IRQHandler(void)
{
  /* USER CODE BEGIN TIM21_IRQn 0 */
  if (tsrCanChangeDirection[tsrIndex]) {
    tsrDirection =
        HAL_GPIO_ReadPin(Reversed_Ph_GPIO_Port, Reversed_Ph_Pin) == GPIO_PIN_RESET ? 1 : 0;
  }
  if (HAL_GPIO_ReadPin(R_off_GPIO_Port, R_off_Pin) == GPIO_PIN_RESET) { // R off
    HAL_GPIO_WritePin(T_push_pull_GPIO_Port, T_push_pull_Pin, GPIO_PIN_RESET); // T low
    HAL_GPIO_WritePin(R_relay_GPIO_Port, R_relay_Pin, GPIO_PIN_SET); // R relay energized
  } else {
    HAL_GPIO_WritePin(T_push_pull_GPIO_Port, T_push_pull_Pin, tsr[tsrDirection][tsrIndex][0]); // T free to run
    HAL_GPIO_WritePin(R_relay_GPIO_Port, R_relay_Pin, GPIO_PIN_RESET); // R relay not energized
  }
  if (HAL_GPIO_ReadPin(S_off_GPIO_Port, S_off_Pin) == GPIO_PIN_RESET) { // S off
    HAL_GPIO_WritePin(R_push_pull_GPIO_Port, R_push_pull_Pin, GPIO_PIN_RESET); // R low
    HAL_GPIO_WritePin(S_relay_GPIO_Port, S_relay_Pin, GPIO_PIN_SET); // S relay energized
  } else {
    HAL_GPIO_WritePin(R_push_pull_GPIO_Port, R_push_pull_Pin, tsr[tsrDirection][tsrIndex][2]); // R free to run
    HAL_GPIO_WritePin(S_relay_GPIO_Port, S_relay_Pin, GPIO_PIN_RESET); // S relay not energized
  }
  if (HAL_GPIO_ReadPin(T_off_GPIO_Port, T_off_Pin) == GPIO_PIN_RESET) { // T off
    HAL_GPIO_WritePin(S_push_pull_GPIO_Port, S_push_pull_Pin, GPIO_PIN_RESET); // S low
    HAL_GPIO_WritePin(T_relay_GPIO_Port, T_relay_Pin, GPIO_PIN_SET); // T relay energized
  } else {
    HAL_GPIO_WritePin(S_push_pull_GPIO_Port, S_push_pull_Pin, tsr[tsrDirection][tsrIndex][1]); // S free to run
    HAL_GPIO_WritePin(T_relay_GPIO_Port, T_relay_Pin, GPIO_PIN_RESET); // T relay not energized
  }
  tsrIndex = (tsrIndex + 1) % 6;
  /* USER CODE END TIM21_IRQn 0 */
  HAL_TIM_IRQHandler(&htim21);
  /* USER CODE BEGIN TIM21_IRQn 1 */

  /* USER CODE END TIM21_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
