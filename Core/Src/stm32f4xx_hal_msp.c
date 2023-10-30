/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32f4xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
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
/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include <stdbool.h>

extern int stopped;

static int rcvdImuInt = 0;
extern bool backButtonSet, powerButtonSet, okButtonSet;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */
	__enable_irq();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}
/**
 * @brief I2C MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hi2c->Instance == I2C1) {
		/* USER CODE BEGIN I2C1_MspInit 0 */

		/* USER CODE END I2C1_MspInit 0 */

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**I2C1 GPIO Configuration
		 PB8     ------> I2C1_SCL
		 PB9     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE();
		/* Peripheral interrupt init */
//        HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
//        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
//        HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
//        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
		/* USER CODE BEGIN I2C1_MspInit 1 */
		/* USER CODE BEGIN I2C1_MspInit 1 */

		/* USER CODE END I2C1_MspInit 1 */
	}

	if (hi2c->Instance == I2C3) {
		/* USER CODE BEGIN I2C3_MspInit 0 */

		/* USER CODE END I2C3_MspInit 0 */

		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**I2C3 GPIO Configuration
		 PC9     ------> I2C3_SDA
		 PA8     ------> I2C3_SCL
		 */
		__HAL_RCC_I2C3_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral clock enable */
//        __HAL_RCC_I2C3_CLK_ENABLE();
		/* I2C3 interrupt Init */
//        HAL_NVIC_SetPriority(I2C3_EV_IRQn, 5, 0);
//        HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
//        HAL_NVIC_SetPriority(I2C3_ER_IRQn, 5, 0);
//        HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
		/* USER CODE BEGIN I2C3_MspInit 1 */

		/* USER CODE END I2C3_MspInit 1 */
	}

}

/**
 * @brief I2C MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hi2c: I2C handle pointer
 * @retval None
 */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c) {

	if (hi2c->Instance == I2C1) {
		/* USER CODE BEGIN I2C1_MspDeInit 0 */

		/* USER CODE END I2C1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C1_CLK_DISABLE();

		/**I2C1 GPIO Configuration
		 PB8     ------> I2C1_SCL
		 PB9     ------> I2C1_SDA
		 */
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8 | GPIO_PIN_9);
		/* Peripheral interrupt DeInit*/
		HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);

		HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);

	}
	/* USER CODE BEGIN I2C1_MspDeInit 1 */

	/* USER CODE END I2C1_MspDeInit 1 */

	if (hi2c->Instance == I2C3) {
		/* USER CODE BEGIN I2C3_MspDeInit 0 */

		/* USER CODE END I2C3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_I2C3_CLK_DISABLE();

		/**I2C3 GPIO Configuration
		 PC9     ------> I2C3_SDA
		 PA8     ------> I2C3_SCL
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);

		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

		/* I2C3 interrupt DeInit */
		HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
		HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
		/* USER CODE BEGIN I2C3_MspDeInit 1 */

		/* USER CODE END I2C3_MspDeInit 1 */
	}

}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (huart->Instance == USART2) {
		/* USER CODE BEGIN USART2_MspInit 0 */

		/* USER CODE END USART2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_USART2_CLK_ENABLE();

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USART2 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA3     ------> USART2_RX
		 */
		GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN USART2_MspInit 1 */

		/* USER CODE END USART2_MspInit 1 */
	}

}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		/* USER CODE BEGIN USART2_MspDeInit 0 */

		/* USER CODE END USART2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USART2_CLK_DISABLE();

		/**USART2 GPIO Configuration
		 PA2     ------> USART2_TX
		 PA3     ------> USART2_RX
		 */
		HAL_GPIO_DeInit(GPIOA, USART_TX_Pin | USART_RX_Pin);

		/* USER CODE BEGIN USART2_MspDeInit 1 */

		/* USER CODE END USART2_MspDeInit 1 */
	}

}

/* USER CODE BEGIN 1 */
/**
 * @brief RTC MSP Initialization
 * This function configures the hardware resources used in this example* @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };
	if (hrtc->Instance == RTC) {
		/* USER CODE BEGIN RTC_MspInit 0 */

		/* USER CODE END RTC_MspInit 0 */
		/** Initializes the peripherals clock
		 */
		PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
		PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV8;
		if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
			Error_Handler();
		}

		/* Peripheral clock enable */
		__HAL_RCC_RTC_ENABLE();
		/* USER CODE BEGIN RTC_MspInit 1 */

		/* USER CODE END RTC_MspInit 1 */
	}

}

/**
 * @brief RTC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hrtc: RTC handle pointer
 * @retval None
 */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc) {
	if (hrtc->Instance == RTC) {
		/* USER CODE BEGIN RTC_MspDeInit 0 */

		/* USER CODE END RTC_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_RTC_DISABLE();
		/* USER CODE BEGIN RTC_MspDeInit 1 */

		/* USER CODE END RTC_MspDeInit 1 */
	}

}

/**
 * @brief PCD MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hpcd: PCD handle pointer
 * @retval None
 */
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hpcd->Instance == USB_OTG_FS) {

		/* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

		/* USER CODE END USB_OTG_FS_MspInit 0 */

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**USB_OTG_FS GPIO Configuration
		 PA11     ------> USB_OTG_FS_DM
		 PA12     ------> USB_OTG_FS_DP
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_USB_OTG_FS_CLK_ENABLE();
		HAL_NVIC_SetPriority(OTG_FS_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
		/* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

		/* USER CODE END USB_OTG_FS_MspInit 1 */
	}

}

/**
 * @brief PCD MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hpcd: PCD handle pointer
 * @retval None
 */
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd) {
	if (hpcd->Instance == USB_OTG_FS) {
		/* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

		/* USER CODE END USB_OTG_FS_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_USB_OTG_FS_CLK_DISABLE();

		/**USB_OTG_FS GPIO Configuration
		 PA11     ------> USB_OTG_FS_DM
		 PA12     ------> USB_OTG_FS_DP
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

		/* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

		/* USER CODE END USB_OTG_FS_MspDeInit 1 */
	}

}

/*******************************************************************************
 LL Driver Callbacks (PCD -> USB Device Library)
 *******************************************************************************/

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_SetupStage(hpcd->pData, (uint8_t*) hpcd->Setup);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_SOF(hpcd->pData);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd) {
	USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
	/*Set USB Current Speed*/
	switch (hpcd->Init.speed) {
	case PCD_SPEED_HIGH:
		speed = USBD_SPEED_HIGH;
		break;

	case PCD_SPEED_FULL:
		speed = USBD_SPEED_FULL;
		break;

	default:
		speed = USBD_SPEED_FULL;
		break;
	}
	USBD_LL_SetSpeed(hpcd->pData, speed);

	/*Reset Device*/
	USBD_LL_Reset(hpcd->pData);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_Suspend(hpcd->pData);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_Resume(hpcd->pData);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum) {
	USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
}

/**
 *
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_DevConnected(hpcd->pData);
}

/**
 * @brief  SOF callback.
 * @param  hpcd: PCD handle
 * @retval None
 */
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd) {
	USBD_LL_DevDisconnected(hpcd->pData);
}

// ----------------------------------------------------------------------------------
// Callbacks for ISR, I2C Operations
// ----------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t n) {
	if (stopped) //during sleep mode
	{
		printf("rcv %d\r\n", rcvdImuInt);
		if ((n == GPIO_PIN_0) && rcvdImuInt) {
			/* enable lontium */
			set_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
//			reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);  // for time being
//			Display_Configuration(0);
			wakeup();

			rcvdImuInt = 0;
		} else if (n == GPIO_PIN_1) {
			printf("gpio 1 int\r\n");
			rcvdImuInt = 1; // received imu int while sleeping
			imu_INT_EVT(); // to start receiving imu data after wakeup
		}
	} else if (n == GPIO_PIN_1) {
		imu_INT_EVT();
	} else if (n == POWER_BUTTON_PIN) {
		printf("Set the power button flag to 1\r\n");
		powerButtonSet = true;
	} else if (n == BACK_BUTTON_PIN) {
		printf("Set the back button flag to 1\r\n");
		backButtonSet = true;
	} else if (n == OK_BUTTON_PIN) {
		printf("Set the ok button flag to 1\r\n");
		okButtonSet = true;
	} else {
		printf("Unsupported interrupt\r\n");
	}
}
/**
 * @brief SPI MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hspi: SPI handle pointer
 *
 * @retval None
 */
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspInit 0 */

		/* USER CODE END SPI2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI2_CLK_ENABLE();

		__HAL_RCC_GPIOC_CLK_ENABLE();
		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**SPI2 GPIO Configuration
		 PB14     ------> SPI2_MISO
		 PB15     ------> SPI2_MOSI
		 PB10     ------> SPI2_SCK
		 PB12     ------> SPI2_NSS
		 */
//    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
		GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
//    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		// GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12;
		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* SPI2 interrupt Init */
//    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
//    HAL_NVIC_EnableIRQ(SPI2_IRQn);
		/* USER CODE BEGIN SPI2_MspInit 1 */

		/* USER CODE END SPI2_MspInit 1 */
	}

	else if (hspi->Instance == SPI1) {

		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**SPI1 GPIO Configuration
		 PA4     ------> SPI1_NSS
		 PA5     ------> SPI1_SCK
		 PA6     ------> SPI1_MISO
		 PA7     ------> SPI1_MOSI
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_5 | GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/* USER CODE BEGIN SPI1_MspInit 1 */

		/* USER CODE END SPI1_MspInit 1 */
		/* SPI2 interrupt Init */
//      HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
//      HAL_NVIC_EnableIRQ(SPI2_IRQn);
//
//      HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
//            HAL_NVIC_EnableIRQ(SPI1_IRQn);
		/* USER CODE BEGIN SPI2_MspInit 1 */

		/* USER CODE END SPI2_MspInit 1 */
	}

}

/**
 * @brief SPI MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hspi: SPI handle pointer
 * @retval None
 */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI2) {
		/* USER CODE BEGIN SPI2_MspDeInit 0 */

		/* USER CODE END SPI2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI2_CLK_DISABLE();

		/**SPI2 GPIO Configuration
		 PC2     ------> SPI2_MISO
		 PC3     ------> SPI2_MOSI
		 PB10     ------> SPI2_SCK
		 PB12     ------> SPI2_NSS
		 */
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2 | GPIO_PIN_3);

		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10 | GPIO_PIN_12);

		/* USER CODE BEGIN SPI2_MspDeInit 1 */

		/* USER CODE END SPI2_MspDeInit 1 */
	}

}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
