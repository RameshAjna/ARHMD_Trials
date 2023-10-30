/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cca02m2_audio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern I2C_HandleTypeDef hi2c3;
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi1;
/* USER CODE END EV */
extern int stopped;
extern int enable_print;
extern int goToSleep;
extern bool powerSavingOn;
extern bool powerUpInterrupt;
/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
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
void HardFault_Handler(void) {
	/* USER CODE BEGIN HardFault_IRQn 0 */

	/* USER CODE END HardFault_IRQn 0 */
	while (1) {
//	  printf("entered hard fault\r\n");
		osDelay(1000);
		/* USER CODE BEGIN W1_HardFault_IRQn 0 */
		/* USER CODE END W1_HardFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
	/* USER CODE BEGIN MemoryManagement_IRQn 0 */

	/* USER CODE END MemoryManagement_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
		/* USER CODE END W1_MemoryManagement_IRQn 0 */
	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {
	/* USER CODE BEGIN BusFault_IRQn 0 */

	/* USER CODE END BusFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_BusFault_IRQn 0 */
		/* USER CODE END W1_BusFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
	/* USER CODE BEGIN UsageFault_IRQn 0 */

	/* USER CODE END UsageFault_IRQn 0 */
	while (1) {
		/* USER CODE BEGIN W1_UsageFault_IRQn 0 */
		/* USER CODE END W1_UsageFault_IRQn 0 */
	}
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {
	/* USER CODE BEGIN DebugMonitor_IRQn 0 */

	/* USER CODE END DebugMonitor_IRQn 0 */
	/* USER CODE BEGIN DebugMonitor_IRQn 1 */

	/* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
 */
void TIM1_UP_TIM10_IRQHandler(void) {
	/* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */
//	if(stopped)
//	{
//		    printf("tim1 int\r\n");
//	}
	/* USER CODE END TIM1_UP_TIM10_IRQn 0 */
	HAL_TIM_IRQHandler(&htim1);
	/* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */

	/* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI1_IRQHandler(void) {
	/* USER CODE BEGIN EXTI1_IRQn 0 */
	/* USER CODE END EXTI1_IRQn 0 */
//	if(stopped | enable_print)
//	if(stopped)
//	{
//	    printf(" imu ext int \r\n");
//	}
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
	/* USER CODE BEGIN EXTI1_IRQn 1 */

	/* USER CODE END EXTI1_IRQn 1 */
}

void EXTI3_IRQHandler(void) {
	static long prevTimeButton = 0;
	if ((HAL_GetTick() - prevTimeButton) >= 200) {
		printf("Power button pressed\r\n");
		prevTimeButton = HAL_GetTick();
		/* If the device is not in sleep mode, set the flag */
		if (powerSavingOn == false) {
			HAL_GPIO_EXTI_IRQHandler(POWER_BUTTON_PIN);
		} else {
			powerUpInterrupt = true;
		}
	} else {
		if (__HAL_GPIO_EXTI_GET_IT(POWER_BUTTON_PIN) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_IT(POWER_BUTTON_PIN);
		}
	}
}

void EXTI4_IRQHandler(void) {
	static long prevTime = 0;
	if ((HAL_GetTick() - prevTime) >= 200) {
		printf("Ok button pressed\r\n");
		prevTime = HAL_GetTick();
		HAL_GPIO_EXTI_IRQHandler(OK_BUTTON_PIN);
	} else {
		if (__HAL_GPIO_EXTI_GET_IT(OK_BUTTON_PIN) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_IT(OK_BUTTON_PIN);
		}
	}
}

void EXTI9_5_IRQHandler(void) {
	static long prevTime = 0;
	if ((HAL_GetTick() - prevTime) >= 400) {
		prevTime = HAL_GetTick();
		uint32_t status = EXTI->PR;
		uint32_t bitMask = (1 << 6);
		if (bitMask & status) {
			printf("Back button pressed\r\n");
			HAL_GPIO_EXTI_IRQHandler(BACK_BUTTON_PIN);
//			printf("Interrupt occured on EXTI 6 line\r\n");
//			EXTI->PR |= bitMask;		// Clearing the interrupt
		}
	} else {
		if (__HAL_GPIO_EXTI_GET_IT(BACK_BUTTON_PIN) != RESET) {
			__HAL_GPIO_EXTI_CLEAR_IT(BACK_BUTTON_PIN);
		}
	}
}

/* USER CODE BEGIN 1 */
void OTG_FS_IRQHandler(void) {
//	if(stopped)
//	    printf("pcd int\r\n");
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/**
 * @brief  This function handles DMA Stream interrupt request.
 * @param  None
 * @retval None
 */
void AUDIO_IN_I2S_IRQHandler(void) {
//	if(stopped)
//	    printf("dma int\r\n");
	HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}

/* USER CODE BEGIN 1 */

/**
 * @brief This function handles I2C3 event interrupt.
 */
void I2C3_EV_IRQHandler(void) {
	/* USER CODE BEGIN I2C3_EV_IRQn 0 */
//	if(stopped)
//		    printf("i2c3 int\r\n");
	/* USER CODE END I2C3_EV_IRQn 0 */
	HAL_I2C_EV_IRQHandler(&hi2c3);
	/* USER CODE BEGIN I2C3_EV_IRQn 1 */

	/* USER CODE END I2C3_EV_IRQn 1 */
}

/**
 * @brief This function handles I2C3 error interrupt.
 */
void I2C3_ER_IRQHandler(void) {
	/* USER CODE BEGIN I2C3_ER_IRQn 0 */

	/* USER CODE END I2C3_ER_IRQn 0 */
	HAL_I2C_ER_IRQHandler(&hi2c3);
	/* USER CODE BEGIN I2C3_ER_IRQn 1 */

	/* USER CODE END I2C3_ER_IRQn 1 */
}

void EXTI0_IRQHandler(void) {
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */
	printf("prox int\r\n");
	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void) {
	/* USER CODE BEGIN EXTI15_10_IRQn 0 */
	/* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
	/* USER CODE BEGIN EXTI15_10_IRQn 1 */

	/* USER CODE END EXTI15_10_IRQn 1 */
}

void SPI2_IRQHandler(void) {
	/* USER CODE BEGIN I2C3_EV_IRQn 0 */

	/* USER CODE END I2C3_EV_IRQn 0 */
	HAL_SPI_IRQHandler(&hspi2);
//  printf("spi int\r\n");
	/* USER CODE BEGIN I2C3_EV_IRQn 1 */

	/* USER CODE END I2C3_EV_IRQn 1 */
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
