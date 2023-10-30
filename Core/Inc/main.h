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
#include "stm32f4xx_hal.h"
#include "audio_application.h"
#include "usbd_smart_glass.h"
#include "cca02m2_audio.h"
#include "app_cfg.h"
#include "prox.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "LVDS_app.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart.h"
#include "stdio.h"
#include "flash.h"
#include <string.h>
#include "app_cfg.h"
#include "stream_buffer.h"
#include "usbd_command_if.h"
#include "queue.h"
#include "semphr.h"
#include "stdbool.h"
/* USER CODE END Includes */
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rtc.h"

#include "uart.h"
#include "stdio.h"
#include "sensor_proximity.h"
#include "I2C_Functions.h"
#include "commands.h"
#include "display_cfg.h"

#define ACC_FLAG	1
#define RV_FLAG		2
#define MAG_FLAG	4
#define GYRO_FLAG	8
#define PROX_INT_FLAG	1
#define PROX_INT_FLAG	2
#define ALL_WAKE_INT	PROX_INT_FLAG|PROX_INT_FLAG

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA

#define SH_INTN_Pin GPIO_PIN_1 // IMU interrupt pin
#define SH_INTN_GPIO_Port GPIOB // IMU interrupt pin port
#define SH_RSTN_Pin GPIO_PIN_0 // IMU reset pin
#define SH_RSTN_GPIO_Port GPIOC // IMU reset pin port
#define SH_BOOTN_Pin GPIO_PIN_1 // IMU Boot pin
#define SH_BOOTN_GPIO_Port GPIOC // IMU Boot pin port
#define IMU_INT_PRIORITY	5

#define BACK_BUTTON_PIN GPIO_PIN_6
#define OK_BUTTON_PIN GPIO_PIN_4
#define POWER_BUTTON_PIN GPIO_PIN_3


#define HUB_RESET_PORT	GPIOC
#define HUB_RESET_PIN	GPIO_PIN_5

#define SLEEP_MODE_ACTIVE	"SMA"
#define SLEEP_MODE_INACTIVE "SMI"

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define PROC_BUFFER_SIZE    100

#define PACK_SIZE 			2

#define RESET 0
#define FALSE 0
#define TRUE 1

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
void printEvent(const sh2_SensorEvent_t *pEvent);
void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent);
void enterSleep();
void exitSleep();
/* USER CODE BEGIN EFP */
typedef enum {
    STATE_IDLE,
    STATE_HEADER,
    STATE_DATA,
    STATE_FOOTER
} IOStates_t;

typedef struct __packed IMU_Data
{
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float mag_x;
    float mag_y;
    float mag_z;
    float RV_i[PACK_SIZE];
    float RV_j[PACK_SIZE];
    float RV_k[PACK_SIZE];
    float RV_real[PACK_SIZE];
    float AR_RV_i[PACK_SIZE];
    float AR_RV_j[PACK_SIZE];
    float AR_RV_k[PACK_SIZE];
    float AR_RV_real[PACK_SIZE];
    uint16_t prox_data;
    uint32_t timestamp;
    uint16_t backButtonFlag;
    uint16_t okButtonFlag;
    uint16_t powerButtonFlag;
    uint16_t powerSavingStatus;
} IMU_Data_t;

IMU_Data_t IMU_SensorData;
void receiveEvent(const sh2_SensorEvent_t *pEvent);
void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent);
void reportProdIds(void);

void enableButtonInterrupts_v(void);
void setDefaultConfigSettings_v(void);
bool checkFirstTimeBoot_b(void);
void readSleepTimeout_v(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
