/**
 ******************************************************************************
 * @file    stm32_mc_common_it.c
 * @author  Motor Control SDK Team, ST Microelectronics
 * @brief   Main Interrupt Service Routines.
 *          This file provides exceptions handler and peripherals interrupt
 *          service routine related to Motor Control for the STM32 Family
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 * @ingroup STM32G4xx_IRQ_Handlers
 */

/* Includes ------------------------------------------------------------------*/
#include "mc_config.h"
#include "mc_type.h"
//cstat -MISRAC2012-Rule-3.1
#include "mc_tasks.h"
//cstat +MISRAC2012-Rule-3.1
#include "parameters_conversion.h"
#include "motorcontrol.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "mcp_config.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/** @addtogroup MCSDK
 * @{
 */
/** @addtogroup STM32G4xx_IRQ_Handlers STM32G4xx IRQ Handlers
 * @{
 */

/* USER CODE BEGIN PRIVATE */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTICK_DIVIDER (SYS_TICK_FREQUENCY/1000U)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern TIM_HandleTypeDef htim6;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* USER CODE END PRIVATE */

void EXTI15_10_IRQHandler(void);
void HardFault_Handler(void);
void SysTick_Handler(void);

void SPD_ENC_TIM_M1_IRQHandler(void);

/**
 * @brief  This function handles TIMx global interrupt request for M1 Speed Sensor.
 * @param  None
 */
void SPD_ENC_TIM_M1_IRQHandler(void) {
	/* USER CODE BEGIN SPD_TIM_M1_IRQn 0 */

	/* USER CODE END SPD_TIM_M1_IRQn 0 */

	/* Encoder Timer UPDATE IT is dynamicaly enabled/disabled, checking enable state is required */
	if (LL_TIM_IsEnabledIT_UPDATE(ENCODER_M1.TIMx) != 0U) {
		if (LL_TIM_IsActiveFlag_UPDATE(ENCODER_M1.TIMx) != 0U) {
			LL_TIM_ClearFlag_UPDATE(ENCODER_M1.TIMx);
			(void) ENC_IRQHandler(&ENCODER_M1);

			/* USER CODE BEGIN M1 ENCODER_Update */

			/* USER CODE END M1 ENCODER_Update */
		} else {
			/* No other IT to manage for encoder config */
		}
	} else {
		/* No other IT to manage for encoder config */
	}

	/* USER CODE BEGIN SPD_TIM_M1_IRQn 1 */

	/* USER CODE END SPD_TIM_M1_IRQn 1 */
}

/* This section is present only when MCP over UART_A is used */
/**
 * @brief  This function handles USART interrupt request.
 * @param  None
 */
//cstat !MISRAC2012-Rule-8.4
void USART2_IRQHandler(void) {
	/* USER CODE BEGIN USART2_IRQHandler 0 */
	HAL_UART_IRQHandler(&huart2);
	/* USER CODE END USART2_IRQHandler 0 */

	/* USER CODE BEGIN USART2_IRQHandler 1 */
//
	/* USER CODE END USART2_IRQHandler 1 */
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 */
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */

	TSK_HardwareFaultTask();

	/* Go to infinite loop when Hard Fault exception occurs */
	while (true) {
		/* Nothing to do */
	}

	/* USER CODE BEGIN HardFault_IRQn 1 */

	/* USER CODE END HardFault_IRQn 1 */
}

void SysTick_Handler(void) {
#ifdef MC_HAL_IS_USED
	static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
	/* USER CODE BEGIN SysTick_IRQn 0 */

	/* USER CODE END SysTick_IRQn 0 */
	if (SystickDividerCounter == SYSTICK_DIVIDER) {
		HAL_IncTick();
		HAL_SYSTICK_IRQHandler();
		SystickDividerCounter = 0;
	} else {
		/* Nothing to do */
	}

	SystickDividerCounter++;
#endif /* MC_HAL_IS_USED */
	/* Buffer is ready by the HW layer to be processed */
	/* NO DMA interrupt */

	/* USER CODE BEGIN SysTick_IRQn 1 */

	/* USER CODE END SysTick_IRQn 1 */

	MC_RunMotorControlTasks();

	/* USER CODE BEGIN SysTick_IRQn 2 */

	/* USER CODE END SysTick_IRQn 2 */
}

/**
 * @brief  This function handles Button IRQ on PIN PC10.

 */
void EXTI15_10_IRQHandler(void) {
	/* USER CODE BEGIN START_STOP_BTN */
	if (0U != LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_10)) {
		LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_10);
		(void) UI_HandleStartStopButton_cb();
	} else {
		/* Nothing to do */
	}

}

/* USER CODE BEGIN 1 */

void DMA1_Channel2_IRQHandler(void)
{
	HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim6);
}

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/

