/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_command_if.h
  * @brief          : Header for usbd_command_if.c file.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_COMMAND_IF_H__
#define __USBD_COMMAND_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_smart_glass.h"
/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* USER CODE BEGIN EXPORTED_DEFINES */


/* USER CODE END EXPORTED_DEFINES */

/* USER CODE BEGIN EXPORTED_TYPES */

 typedef struct __packed {
 uint8_t start_flag;
 uint8_t pck_type;
 } setup_packet_t;

 typedef struct __packed {
 uint8_t pck_type;
 uint8_t cmd_id;
 uint16_t param_len;
 } cmd_data_packet_t;

 typedef struct __packed {
 uint8_t ack_id;
 uint8_t ack_status;
 uint16_t ack_data[2];
 } ack_packet_t;

 typedef enum packet_type {
 CMD_INIT = 0x10,
 CMD_DATA,
 FW_UP_REQ,
 PARAM_PACK,
 } packet_type_t;

 typedef enum command_id {
 SET_PROX_TH = 0x30,
 GET_PROX_TH,
 PROX_ENABLE,
 PROX_DATA,
 IMU_ENABLE,
 IMU_CALIB,
 IMU_SAVE_CALIB,
 IMU_DATA,
 SET_RTC,
 SYS_REBOOT,
 BACKL_SET,
 BACKL_GET,
 TRIG_SYS_UP,
 TRIG_LT_UP,
 TIM_OUT,
 VID_FORMAT,
 SLEEP_MODE_ACTIVATE,
 SLEEP_MODE_DEACTIVATE,
 CLEAR_BACK_BUTTON_FLAG,
 CLEAR_POWER_BUTTON_FLAG,
 CLEAR_OK_BUTTON_FLAG,
 } command_id_t;

 typedef enum acknowledgement_status {
 ACK_SUCCESS = 0x20,
 ACK_UNKNOWN_ID,
 ACK_UNKNOWN_PKT,
 ACK_INVALID_PARAM
 } acknowledgement_status_t;


/* USER CODE END EXPORTED_TYPES */

/* USER CODE BEGIN EXPORTED_MACRO */

/* USER CODE END EXPORTED_MACRO */

/** Template_IF Interface callback. */

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/* Exported functions ------------------------------------------------------- */
uint8_t parse_buffer(uint8_t *buffer, size_t *count);
uint8_t GPIO_toggle(void);

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_COMMAND_IF_H__ */

/****END OF FILE****/
