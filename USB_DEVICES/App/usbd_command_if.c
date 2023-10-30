/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_command_if.c
  * @author			: vvdn
  ******************************************************************************
  */
 /* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_command_if.h"
#include "main.h"
#include "commands.h"
#include "LVDS_app.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */
extern int goToSleep;
extern int startMonitoringValue;
extern int stopped;
extern bool powerSavingOn;
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

#define NO_COMMAND_APP

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static uint8_t packet_stage = CMD_INIT;
extern int noAcknowledgment;
/* USER CODE END PV */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}Private_Define
  */
/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @} Private_Macros
  */
/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}Private_Variables
  */
/* USER CODE BEGIN PRIVATE_VARIABLES */
uint32_t value = 0;
char backlight = 0;
char string_buffer[10] = {0};
char string_buffer1[10] = {0};
uint32_t crc = 0;
uint32_t crc_size = 0;

uint8_t buf[10] = {0};
/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

extern USBD_HandleTypeDef hUsbDeviceFS;
extern long LTFwSize;
extern uint16_t low_th;
extern uint16_t high_th;
extern QueueHandle_t eventQueue;
extern bool backButtonSet;
extern bool okButtonSet;
extern bool powerButtonSet;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/**
  * @Private functions declaration.
  * {
  */


/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */


/* Private functions ---------------------------------------------------------*/


/**
  *@brief	parse_buffer
			parse the values in the buffer
  *@param	packet_stage : pointer to current state
  *@param	buffer : pointer to the buffer containing data
  *@param	count : pointer to length of the data in buffer
  *@retval	0 if all operations are OK else 1
  */
uint8_t parse_buffer(uint8_t *buffer, size_t *count)
{
#ifndef NO_COMMAND_APP
	return 0;
#else

	if(buffer == NULL || count == NULL){
		return 1;
	}

	uint16_t retval = 0;
	char arg1 = 0;
	char arg2 = 0;
	int enable = 0;
	uint16_t th = 0;
	setup_packet_t setup_packet;
	cmd_data_packet_t cmd_data_packet;
	ack_packet_t ack_packet = {.ack_id = 0, .ack_status = 0, .ack_data[0] = 0, .ack_data[1] = 0};

	static uint8_t sParam_len = 0;

	switch(packet_stage){
	case CMD_INIT:
		if(*count != sizeof(setup_packet)){
			ack_packet.ack_id = ACK_UNKNOWN_PKT;
			ack_packet.ack_status = ACK_UNKNOWN_PKT;
			break;
		}

		memcpy((uint8_t *)&setup_packet, buffer, *count);
		if(setup_packet.start_flag != 0xFF || setup_packet.pck_type != CMD_INIT){
			ack_packet.ack_id = ACK_UNKNOWN_PKT;
			ack_packet.ack_status = ACK_UNKNOWN_PKT;
			break;
		}

		ack_packet.ack_id = CMD_INIT;
		ack_packet.ack_status = ACK_SUCCESS;
		ack_packet.ack_data[1] = 0;
		packet_stage = CMD_DATA;
		noAcknowledgment = 0;
		break;

	case CMD_DATA:
		if(*count < sizeof(cmd_data_packet)){
			ack_packet.ack_id = ACK_UNKNOWN_PKT;
			ack_packet.ack_status = ACK_UNKNOWN_PKT;
			packet_stage = CMD_INIT;
			break;
		}

		memcpy((uint8_t *)&cmd_data_packet, buffer, sizeof(cmd_data_packet));

		if(cmd_data_packet.pck_type != CMD_DATA && cmd_data_packet.pck_type != FW_UP_REQ){
			ack_packet.ack_id = ACK_UNKNOWN_PKT;
			ack_packet.ack_status = ACK_UNKNOWN_PKT;
			packet_stage = CMD_INIT;
			break;
		}

		sParam_len = cmd_data_packet.param_len;
		if(cmd_data_packet.pck_type == FW_UP_REQ){
			ack_packet.ack_id = FW_UP_REQ;

			switch(cmd_data_packet.cmd_id){
			case TRIG_SYS_UP:
				if(sParam_len < 8){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				memcpy((uint8_t *)&crc, buffer + sizeof(cmd_data_packet), 4);
				memcpy((uint8_t *)&value, buffer + sizeof(cmd_data_packet) + sizeof(uint32_t), 4);
				sprintf(string_buffer, "%ld", crc);
				sprintf(string_buffer1, "%ld", value);
				enable_acc_t(&enable, &enable, &enable);
				enable_gyro_t(&enable, &enable, &enable);
				enable_mag_t(&enable, &enable, &enable);
				enable_RV_t(&enable, &enable, &enable);
				trigger_sys_upgrade_t(string_buffer, string_buffer1, NULL);
				break;
			case TRIG_LT_UP:
				if(sParam_len < 4){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				memcpy((uint8_t *)&value, buffer + sizeof(cmd_data_packet), 4);
				LTFwSize = value;
				disable_sensors();
				I2C1_clock_factor(1);
				retval = LVDS_FWU();
				printf("plz reboot the board\r\n");
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			default:
				ack_packet.ack_status = ACK_UNKNOWN_ID;
			}
		}
		if(cmd_data_packet.pck_type == CMD_DATA){
			ack_packet.ack_id = CMD_DATA;

			switch(cmd_data_packet.cmd_id){
			case SET_PROX_TH:
				if(sParam_len < 4){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				memcpy((uint8_t *)&th, buffer + sizeof(cmd_data_packet), 2);
				sprintf(string_buffer, "%d", th);
				memcpy((uint8_t *)&th, buffer + sizeof(cmd_data_packet) + sizeof(th), 2);
				sprintf(string_buffer1, "%d", th);
				retval = prox_set_TH_t(string_buffer, string_buffer1, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case GET_PROX_TH:
				retval = prox_get_TH_t(&arg1, &arg2, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[0] = low_th;
				ack_packet.ack_data[1] = high_th;
				packet_stage = CMD_INIT;
				break;
			case SET_RTC:
				if(sParam_len < 4){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				memcpy((uint8_t *)&value, buffer + sizeof(cmd_data_packet), 4);
				sprintf(string_buffer, "%d", (int)value);
				retval = rtc_set_t(string_buffer, &arg2, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case PROX_ENABLE:
				if(sParam_len < 1){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				arg1 = *(buffer + sizeof(cmd_data_packet));
				retval = prox_enable_t(&arg1, &arg2, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case IMU_ENABLE:
				if(sParam_len < 1){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				arg1 = *(buffer + sizeof(cmd_data_packet));
				retval = enable_acc_t(&arg1, &arg2, NULL);
				retval &= enable_gyro_t(&arg1, &arg2, NULL);
				retval &= enable_mag_t(&arg1, &arg2, NULL);
				retval &= enable_RV_t(&arg1, &arg2, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case IMU_CALIB:
				retval = imu_calibrate_t(NULL, NULL, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				packet_stage = CMD_INIT;
				break;
			case IMU_SAVE_CALIB:
				retval = imu_cal_save_t(NULL, NULL, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				packet_stage = CMD_INIT;
				break;
			case SYS_REBOOT:
				reboot_t(NULL, NULL, NULL);
				break;
			case BACKL_SET:
				//value TBD
				if(sParam_len < 1){
					ack_packet.ack_status = ACK_INVALID_PARAM;
					break;
				}
				backlight = *(buffer + sizeof(cmd_data_packet));
				sprintf(string_buffer, "%d", backlight);
				retval = set_backlight_t(string_buffer, NULL, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case BACKL_GET:
				retval = get_backlight_t(NULL, NULL, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = (uint8_t)retval;
				packet_stage = CMD_INIT;
				break;
			case PROX_DATA:
				retval = prox_get_data_t(&arg1, NULL, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = (uint16_t)retval;
				packet_stage = CMD_INIT;
				break;
			case IMU_DATA:
				memcpy(buffer, (uint8_t *)&IMU_SensorData, sizeof(IMU_SensorData)-6);
				*count = sizeof(IMU_SensorData)-6;
				packet_stage = CMD_INIT;
				return 0;
				break;
			case TIM_OUT:
				value = (uint32_t)buffer[0 + sizeof(cmd_data_packet)];
				sprintf(string_buffer, "%d", (int)value);
				retval = set_time_out_t(string_buffer, &arg2, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case VID_FORMAT:
				value = (uint32_t)buffer[0 + sizeof(cmd_data_packet)];
				sprintf(string_buffer, "%d", (int)value);
				retval = set_format_t(string_buffer, &arg2, NULL);
				ack_packet.ack_status = ACK_SUCCESS;
				ack_packet.ack_data[1] = retval;
				break;
			case SLEEP_MODE_ACTIVATE:
				printf("Sleeping..\r\n");
				startMonitoringValue = 0;
				enterSleep();
				noAcknowledgment = 1;
				break;
			case SLEEP_MODE_DEACTIVATE:
				exitSleep();
				noAcknowledgment = 1;
				break;
			case CLEAR_BACK_BUTTON_FLAG:
				IMU_SensorData.backButtonFlag = 0;
				noAcknowledgment = 1;
				break;
			case CLEAR_OK_BUTTON_FLAG:
				IMU_SensorData
				.okButtonFlag = 0;
				noAcknowledgment = 1;
				break;
			case CLEAR_POWER_BUTTON_FLAG:
				IMU_SensorData.powerButtonFlag = 0;
				noAcknowledgment = 1;
				break;
			default:
				ack_packet.ack_status = ACK_UNKNOWN_ID;
			}
		}
		packet_stage = CMD_INIT;
		break;
	case 4:
		break;

	default:
		break;
	}

	memcpy(buffer, (uint8_t *)&ack_packet, sizeof(ack_packet));
	*count = sizeof(ack_packet);
#endif
	return 0;
}

/****END OF FILE****/
