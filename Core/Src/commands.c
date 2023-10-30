/*
 * commands.c
 *
 *  Created on: 19-Oct-2021
 *      Author: vvdn
 */

#include "commands.h"

#define APP_VERSION				"1.0.9"
#define BOOTLOADER_VERSION		"1.000"
#define CRC_CHECK_0		0
#define CRC_CHECK_1		1

extern int sleepTimeout;
extern I2C_HandleTypeDef hi2c3;

extern uint16_t low_th;
extern uint16_t high_th;

/*@brief	hextoint
 *@param	hex : pointer to hex array
 */
uint32_t hextoint(char *hex) {
	uint32_t val = 0;
	while (*hex) {
		// get current character then increment
		uint8_t byte = *hex++;
		// transform hex character to the 4bit equivalent number, using the ascii table indexes
		if (byte >= '0' && byte <= '9')
			byte = byte - '0';
		else if (byte >= 'a' && byte <= 'f')
			byte = byte - 'a' + 10;
		else if (byte >= 'A' && byte <= 'F')
			byte = byte - 'A' + 10;
		// shift 4 to make space for new digit, and add the 4 bits of the new digit
		val = (val << 4) | (byte & 0xF);
	}

	return val;
}

void disable_sensors(void) {
	int enable = 0;

	/*disable IMU sensor*/
	enable_acc_t(&enable, &enable, &enable);
	enable_gyro_t(&enable, &enable, &enable);
	enable_mag_t(&enable, &enable, &enable);
	enable_RV_t(&enable, &enable, &enable);

	/*disable prox sensor*/
	prox_read();
	stop_Prox_Sensor();
	HAL_I2C_DeInit(&hi2c3);
}

const CLI_CMD_T clicommands[MAX_NUM_CMDS] = { { "reboot", 0, reboot_t }, {
		"imu_read_data", 1, imu_read_data_t },
		{ "enable_acc", 1, enable_acc_t }, { "enable_mag", 1, enable_mag_t }, {
				"enable_gyro", 1, enable_gyro_t },
		{ "enable_RV", 1, enable_RV_t },
		{ "imu_calibrate", 0, imu_calibrate_t }, { "imu_cal_save", 0,
				imu_cal_save_t }, { "prox_enable", 1, prox_enable_t }, {
				"prox_set_TH", 2, prox_set_TH_t }, { "prox_get_TH", 0,
				prox_get_TH_t }, { "prox_get_data", 0, prox_get_data_t }, {
				"set_backlight", 1, set_backlight_t }, { "get_backlight", 1,
				get_backlight_t }, { "trigger_sys_upgrade", 0,
				trigger_sys_upgrade_t }, { "rtc_set", 1, rtc_set_t }, {
				"rtc_get", 1, rtc_get_t }, { "gpio_read_pin", 2, gpio_read_pin_t

		}, { "gpio_write_pin", 3, gpio_write_pin_t }, { "version_read", 0,
				version_read_t }, { "set_time_out", 1, set_time_out_t }, {
				"set_format", 1, set_format_t }, { "i2c_read", 2, i2c_read_t },
		{ "i2c_write", 3, i2c_write_t },
		{ "set_board_info", 3, set_board_info_t }, { "get_board_info", 0,
				get_board_info_t }, { "set_luminous", 1, set_luminous_t }, {
				"get_luminous", 1, get_luminous_t },

};

int reboot_t(char *arg1, char *arg2, char *arg3) {
	reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
	osDelay(100);
	disable_sensors();

	printf("\r\n\n****rebooted***\r\n");
	NVIC_SystemReset();

	return SUCCESS;
}

int imu_read_data_t(char *arg1, char *arg2, char *arg3) {
	printf("imu_read_data_t\r\n");

	if ((strncmp(arg1, "acc", 3)) == 0) {
		printf("Raw acc: x %f y %f z %f\r\n", IMU_SensorData.acc_x,
				IMU_SensorData.acc_y, IMU_SensorData.acc_z);
	} else if ((strncmp(arg1, "gyro", 4)) == 0) {
		printf("Raw gyro: x %f y %f z %f\r\n", IMU_SensorData.gyro_x,
				IMU_SensorData.gyro_y, IMU_SensorData.gyro_z);
	} else if ((strncmp(arg1, "mag", 3)) == 0) {
		printf("Raw mag: x %f y %f z %f\r\n", IMU_SensorData.mag_x,
				IMU_SensorData.mag_y, IMU_SensorData.mag_z);
	} else if ((strncmp(arg1, "RV", 2)) == 0) {
		printf("RV: i %f j %f k %f real %f\r\n"
				"AR_RV: i %f j %f k %f real %f\r\n", IMU_SensorData.RV_i[0],
				IMU_SensorData.RV_j[0], IMU_SensorData.RV_k[0],
				IMU_SensorData.RV_real[0], IMU_SensorData.AR_RV_i[0],
				IMU_SensorData.AR_RV_j[0], IMU_SensorData.AR_RV_k[0],
				IMU_SensorData.AR_RV_real[0]);
	} else {
		printf("Invalid Argument\r\n");
		return FAILURE;
	}
	return SUCCESS;
}

int enable_acc_t(char *arg1, char *arg2, char *arg3) {
	sh2_SensorConfig_t config_sensor;
	int status = 1;

	config_sensor.changeSensitivityEnabled = false;
	config_sensor.wakeupEnabled = false;
	config_sensor.changeSensitivityRelative = false;
	config_sensor.alwaysOnEnabled = false;
	config_sensor.changeSensitivity = 0;
	config_sensor.batchInterval_us = 0;

	if (((strcmp(arg1, "0")) == 0) || (*arg1 == 0)) {
		//change the report interval
		config_sensor.reportInterval_us = 0;  // To disable the sensor

		status = sh2_setSensorConfig(SH2_ACCELEROMETER, &config_sensor);

		HAL_Delay(100);
		IMU_SensorData.acc_x = 0;
		IMU_SensorData.acc_y = 0;
		IMU_SensorData.acc_z = 0;
		printf("accelerometer disabled\r\n");
	} else if (((strcmp(arg1, "1")) == 0) || (*arg1 == 1)) {
		config_sensor.reportInterval_us = 10000;  // microseconds (100 Hz)
		status = sh2_setSensorConfig(SH2_ACCELEROMETER, &config_sensor);
		printf("accelerometer enabled\r\n");
	} else {
		printf("Invalid Argument\r\n");

		return FAILURE;
	}

	if (status != HAL_OK) {
		return FAILURE;
	}

	return SUCCESS;
}

int enable_gyro_t(char *arg1, char *arg2, char *arg3) {
	sh2_SensorConfig_t config_sensor;
	int status = 1;

	config_sensor.changeSensitivityEnabled = false;
	config_sensor.wakeupEnabled = false;
	config_sensor.changeSensitivityRelative = false;
	config_sensor.alwaysOnEnabled = false;
	config_sensor.changeSensitivity = 0;
	config_sensor.batchInterval_us = 0;

	if (((strcmp(arg1, "0")) == 0) || (*arg1 == 0)) { //change the report interval
		config_sensor.reportInterval_us = 0;  // To disable the sensor
		status = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config_sensor);

		HAL_Delay(100);
		IMU_SensorData.gyro_x = 0;
		IMU_SensorData.gyro_y = 0;
		IMU_SensorData.gyro_z = 0;
		printf("gyroscope disabled\r\n");

	} else if (((strcmp(arg1, "1")) == 0) || (*arg1 == 1)) {
		config_sensor.reportInterval_us = 10000;  // microseconds (100 Hz)
		status = sh2_setSensorConfig(SH2_GYROSCOPE_CALIBRATED, &config_sensor);
		printf("gyroscope enabled\r\n");

	} else {
		printf("Invalid Argument\r\n");

		return FAILURE;
	}

	if (status != HAL_OK) {
		return FAILURE;
	}
	return SUCCESS;
}

int enable_mag_t(char *arg1, char *arg2, char *arg3) {
	sh2_SensorConfig_t config_sensor;
	int status = 1;

	config_sensor.changeSensitivityEnabled = false;
	config_sensor.wakeupEnabled = false;
	config_sensor.changeSensitivityRelative = false;
	config_sensor.alwaysOnEnabled = false;
	config_sensor.changeSensitivity = 0;
	config_sensor.batchInterval_us = 0;

	if (((strcmp(arg1, "0")) == 0) || (*arg1 == 0)) { //change the report interval
		config_sensor.reportInterval_us = 0;  // To disable the sensor
		status = sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED,
				&config_sensor);

		HAL_Delay(100);
		IMU_SensorData.mag_x = 0;
		IMU_SensorData.mag_y = 0;
		IMU_SensorData.mag_z = 0;
		printf("magnetometer disabled\r\n");

	} else if (((strcmp(arg1, "1")) == 0) || (*arg1 == 1)) {
		config_sensor.reportInterval_us = 10000;  // microseconds (100 Hz)
		status = sh2_setSensorConfig(SH2_MAGNETIC_FIELD_CALIBRATED,
				&config_sensor);
		printf("magnetometer enabled\r\n");

	} else {
		printf("Invalid Argument\r\n");

		return FAILURE;
	}

	if (status != HAL_OK) {
		return FAILURE;
	}
	return SUCCESS;
}

int enable_RV_t(char *arg1, char *arg2, char *arg3) {
	sh2_SensorConfig_t config_sensor;
	int status = 1;

	config_sensor.changeSensitivityEnabled = false;
	config_sensor.wakeupEnabled = false;
	config_sensor.changeSensitivityRelative = false;
	config_sensor.alwaysOnEnabled = false;
	config_sensor.changeSensitivity = 0;
	config_sensor.batchInterval_us = 0;

	if (((strcmp(arg1, "0")) == 0) || (*arg1 == 0)) { //change the report interval
		config_sensor.reportInterval_us = 0;  // To disable the sensor
		status = sh2_setSensorConfig(SH2_ARVR_STABILIZED_RV, &config_sensor);
		if (status != HAL_OK) {
			printf("ARVR_RV not configured\r\n");
		}
		status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config_sensor);
		if (status != HAL_OK) {
			printf("RV not configured\r\n");
		}

		HAL_Delay(100);
		IMU_SensorData.RV_i[0] = 0;
		IMU_SensorData.RV_j[0] = 0;
		IMU_SensorData.RV_k[0] = 0;
		IMU_SensorData.RV_real[0] = 0;

		IMU_SensorData.AR_RV_i[0] = 0;
		IMU_SensorData.AR_RV_j[0] = 0;
		IMU_SensorData.AR_RV_k[0] = 0;
		IMU_SensorData.AR_RV_real[0] = 0;
		printf("RV disabled\r\n");

	} else if (((strcmp(arg1, "1")) == 0) || (*arg1 == 1)) {
		config_sensor.reportInterval_us = 2500;  // microseconds (400 Hz)
		status = sh2_setSensorConfig(SH2_ARVR_STABILIZED_RV, &config_sensor);
		status = sh2_setSensorConfig(SH2_ROTATION_VECTOR, &config_sensor);
		printf("RV enabled\r\n");

	} else {
		printf("Invalid Argument\r\n");

		return FAILURE;
	}

	if (status != HAL_OK) {
		return FAILURE;
	}
	return SUCCESS;
}

int imu_calibrate_t(char *arg1, char *arg2, char *arg3) {
	int status = 0;

	status = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
	if (status != HAL_OK) {
		printf("Error: %d, from sh2_setCalConfig() in configure().\n", status);
	} else {
		printf("calibration done\r\n");
	}

	return SUCCESS;
}

int imu_cal_save_t(char *arg1, char *arg2, char *arg3) {
	int status = 0;

	status = sh2_saveDcdNow();

	if (status != HAL_OK) {
		printf("Error: %d, from sh2_saveDcdNow() in saving the calibration.\n",
				status);
	} else {
		printf("calibration saved\r\n");
	}
	return SUCCESS;

}

int prox_enable_t(char *arg1, char *arg2, char *arg3) {
	if ((strcmp(arg1, "1") == 0) || (*arg1 == 1)) {
		if (start_Prox_Sensor() == OK) {
			printf("Prox: Enabled\r\n");
			return SUCCESS;
		}

		printf("Prox: Enable Failed\r\n");
		return FAILURE;
	}
	if ((strcmp(arg1, "0") == 0) || (*arg1 == 0)) {
		if (stop_Prox_Sensor() == OK) {
			IMU_SensorData.prox_data = 0;
			printf("Prox: Disabled\r\n");
			return SUCCESS;
		}
		printf("Prox: Disable Failed\r\n");
		return FAILURE;
	}

	printf("Prox: Wrong Parameter\r\n");

	return FAILURE;
}

int prox_set_TH_t(char *arg1, char *arg2, char *arg3) {
	uint16_t l_th = 0;
	uint16_t h_th = 0;

	l_th = atoi(arg1);
	h_th = atoi(arg2);

	if ((l_th > h_th) || (l_th > 4095) || (h_th > 4095)) {
		printf("Prox-Threshold: Invalid Parameter\r\n");
		return FAILURE;
	}
	low_th = l_th;
	high_th = h_th;

	printf("Prox-Threshold: Set\r\n");
	return SUCCESS;
}

int prox_get_TH_t(char *arg1, char *arg2, char *arg3) {
	printf("Prox: Low Threshold-%d, High Threshold-%d\r\n", low_th, high_th);

	return SUCCESS;
}

int prox_get_data_t(char *arg1, char *arg2, char *arg3) {
	int retval = IMU_SensorData.prox_data;
	if (retval == PROX_DUMMY_DATA) {
		printf("prox sensor not enabled\r\n");
	} else {
		printf("Prox: Data-%d\r\n", IMU_SensorData.prox_data);
	}

	return retval;
}

int set_backlight_t(char *arg1, char *arg2, char *arg3) {

	int brightness = atoi(arg1);
	if (brightness > 255 || brightness < 0) {
		printf("Invalid argument\r\n");
		return FAILURE;
	}
	printf("Set Brightness %d\r\n", brightness);
	Display_Set_Brightness(brightness);
	return SUCCESS;
}

int get_backlight_t(char *arg1, char *arg2, char *arg3) {
	int brightness = 0;
	Display_Get_Brightness(&brightness);
	printf("Brightness: %d\r\n", brightness);

	return brightness;
}

int set_luminous_t(char *arg1, char *arg2, char *arg3) {

	int lumin = atoi(arg1);
	if (lumin > 255 || lumin < 0) {
		printf("Invalid argument\r\n");
		return FAILURE;
	}
	Display_Set_Luminous(lumin);
	printf("Set Luminance %d\r\n", lumin);
	return SUCCESS;
}

int get_luminous_t(char *arg1, char *arg2, char *arg3) {
	int lumin = 0;
	Display_Get_Luminous(&lumin);
	printf("Luminance: %d\r\n", lumin);

	return lumin;
}

int trigger_sys_upgrade_t(char *arg1, char *arg2, char *arg3) {
	uint32_t crc = 0;
	uint32_t size = 0;
	uint8_t buf[10] = { 0 };
	system_info_t sys_info_read;

	crc = (uint32_t) atoll(arg1);
	size = (uint32_t) atol(arg2);
//	if (crc == 0 || size == 0) {
//		printf("Bad parameter\r\n");
//		return FAILURE;
//	}

	flash_read_FRU(&sys_info_read);
	sys_info_read.crc_arr[0] = crc;
	sys_info_read.crc_arr[1] = size;
	sys_info_read.crc_arr[2] = (uint32_t) CRC_CHECK_0;
	//	crc_buf[0] = crc;
	//	crc_buf[1] = size;
	//	crc_buf[2] = (uint32_t)CRC_CHECK_0;

#if 1
	/* for testing */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x05;
	buf[3] = 0x00;
	buf[4] = 0x01;
	buf[5] = 0x08;
	buf[6] = 0x36;
	buf[7] = 0x02;	//setting as 1

	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 8,
			100)) {
		printf("GPIO set failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
		printf("GPIO set command execute failed\r\n");
	}
#endif

//	Flash_Write_CRC(crc_buf);
//	printf("crc value: %u, size: %u, flag: %ld", sys_info_read.crc_arr[0],
//			sys_info_read.crc_arr[1], sys_info_read.crc_arr[2]);
//	Flash_Write_FRU(&sys_info_read);
//	Flash_CRC_Write(CRC_ADDRESS, crc_buf, (uint16_t)3);

	//disable LT
	reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
	osDelay(100);
	disable_sensors();

	printf("Trigger_Upgrade: Rebooting\r\n");
	NVIC_SystemReset();

	return FAILURE;
}

int rtc_set_t(char *arg1, char *arg2, char *arg3) {
	uint32_t cur_time = 0;
	RTC_DateTypeDef dateStruct;
	RTC_TimeTypeDef timeStruct;
	extern RTC_HandleTypeDef hrtc;

	cur_time = atoi(arg1);
	RTC_FromEpoch(cur_time, &timeStruct, &dateStruct);
	if (HAL_RTC_SetTime(&hrtc, &timeStruct, RTC_FORMAT_BIN) != HAL_OK) {
		printf("RTC: set failed\r\n");
		return ERROR;
	}
	if (HAL_RTC_SetDate(&hrtc, &dateStruct, RTC_FORMAT_BIN) != HAL_OK) {
		printf("RTC: set failed\r\n");
		return ERROR;
	}
	printf("RTC: Time set\r\n");

	return SUCCESS;
}

int rtc_get_t(char *arg1, char *arg2, char *arg3) {
	uint32_t cur_time = 0;
	cur_time = rtc_read();
	printf("RTC: %ld\r\n", cur_time);

	return SUCCESS;
}

int gpio_read_pin_t(char *arg1, char *arg2, char *arg3) {
	int ret = 1;
	char *pin = NULL;
	uint16_t GPIO_pin = (uint16_t) strtol(arg2, &pin, 10);

	if ((GPIO_pin < 0) || (GPIO_pin > 15) || (*pin != '\0')) {
		printf("Invalid pin number\r\nValue should be between 0 to 15\r\n");
		return FAILURE;
	}

	GPIO_pin = 1 << GPIO_pin;

	if (((strcmp(arg1, "A")) == 0)) {
		ret = HAL_GPIO_ReadPin(GPIOA, GPIO_pin);
	} else if (((strcmp(arg1, "B")) == 0)) {
		ret = HAL_GPIO_ReadPin(GPIOB, GPIO_pin);
	} else if (((strcmp(arg1, "C")) == 0)) {
		ret = HAL_GPIO_ReadPin(GPIOC, GPIO_pin);
	} else {
		printf("Invalid parameters\r\n");
		return FAILURE;
	}
	printf("Pin state: %d\r\n", ret);

	return SUCCESS;
}

int gpio_write_pin_t(char *arg1, char *arg2, char *arg3) {
	char *pin = NULL;
	uint16_t GPIO_pin = (uint16_t) strtol(arg2, &pin, 10);

	if ((GPIO_pin < 0) || (GPIO_pin > 15) || (*pin != '\0')) {
		printf("Invalid pin number\r\nvalue should be between 0 to 15\r\n");
		return FAILURE;
	}
	GPIO_pin = 1 << GPIO_pin;

	if (((strcmp(arg1, "A")) == 0)) {
		if ((strcmp(arg3, "0")) == 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_pin, GPIO_PIN_RESET);
			printf("Pin reset done\r\n");
		} else if ((strcmp(arg3, "1")) == 0) {
			HAL_GPIO_WritePin(GPIOA, GPIO_pin, GPIO_PIN_SET);
			printf("Pin set done\r\n");
		} else {
			printf("invalid pin state\r\n  value should be 0 or 1\r\n");
			return FAILURE;
		}
	} else if (((strcmp(arg1, "B")) == 0)) {
		if ((strcmp(arg3, "0")) == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_pin, GPIO_PIN_RESET);
			printf("Pin reset done\r\n");
		} else if ((strcmp(arg3, "1")) == 0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_pin, GPIO_PIN_SET);
			printf("Pin set done\r\n");
		} else {
			printf("invalid pin state\r\n  value should be 0 or 1\r\n");
			return FAILURE;
		}
	} else if (((strcmp(arg1, "C")) == 0)) {
		if ((strcmp(arg3, "0")) == 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_pin, GPIO_PIN_RESET);
			printf("Pin reset done\r\n");
		} else if ((strcmp(arg3, "1")) == 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_pin, GPIO_PIN_SET);
			printf("Pin set done\r\n");
		} else {
			printf("invalid pin state\r\n  value should be 0 or 1\r\n");
			return FAILURE;
		}
	} else {
		printf("invalid parameters\r\n");
		return FAILURE;
	}

	return SUCCESS;
}

int version_read_t(char *arg1, char *arg2, char *arg3) {
//    system_info_t info;
//    flash_read_FRU(&info);
	printf("Bootloader Version: %s, Application Version: %s\r\n",
			BOOTLOADER_VERSION, APP_VERSION);

	return SUCCESS;
}

int set_time_out_t(char *arg1, char *arg2, char *arg3) {
	system_info_t sys_info;
	int value = atoi(arg1);
	if (value > 255 || value < 0) {
		printf("Invalid parameter\r\n");
		return FAILURE;
	}
	flash_read_FRU(&sys_info);
	sys_info.config[SLEEP_TIM] = (uint32_t) value;
	Flash_Write_FRU(&sys_info);
	sleepTimeout = (value) * 1000;
	printf("Time Out: %d ms set\r\n", sleepTimeout);
	return SUCCESS;
}

int set_format_t(char *arg1, char *arg2, char *arg3) {
	uint8_t format = 0;

	format = atoi(arg1);
	if (format == 0
			&& (get_gpio_pin_status(LT_GPIO1_PORT, LT_GPIO1_PIN)
					!= GPIO_PIN_RESET)) {
		/* reset lontium */
		reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);

		reset_gpio_pin(LT_GPIO1_PORT, LT_GPIO1_PIN);

		osDelay(10);
		/* enable lontium */
		set_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
		printf("Format: %d set\r\n", format);

	} else if (format == 1
			&& (get_gpio_pin_status(LT_GPIO1_PORT, LT_GPIO1_PIN) != GPIO_PIN_SET)) {
		/* reset lontium */
		reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);

		set_gpio_pin(LT_GPIO1_PORT, LT_GPIO1_PIN);

		osDelay(10);
		/* enable lontium */
		set_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
		printf("Format: %d set\r\n", format);
	}
	/* otherwise requested state is already there */
	else if (format != 0 && format != 1) {
		printf("Invalid format\r\n");
	}

	return SUCCESS;
}

int i2c_read_t(char *arg1, char *arg2, char *arg3) {
	char *endptr = NULL;
	struct TransferData Data;
	char dev = (char) strtol(arg1, &endptr, 0);

	if (dev == 0x60) {
		char reg = (char) strtol(arg2, &endptr, 0);
		if (*endptr != '\0' || arg2 == endptr) {
			printf("Invalid Register Address\r\n");
			return FAILURE;
		}
		switch (reg) {
		case ST_CONF:
			Data.RegisterAddress = ST_CONF;
			break;
		case PS_CONF1:
			Data.RegisterAddress = PS_CONF1;
			break;
		case PS_CONF3:
			Data.RegisterAddress = PS_CONF3;
			break;
		case PS_THDL:
			Data.RegisterAddress = PS_THDL;
			break;
		case PS_THDH:
			Data.RegisterAddress = PS_THDH;
			break;
		case PS_CANC:
			Data.RegisterAddress = PS_CANC;
			break;
		case PS_AC_LPPERI:
			Data.RegisterAddress = PS_AC_LPPERI;
			break;
		case PS_DATA:
			Data.RegisterAddress = PS_DATA;
			break;
		case VCNL36826S_FLAG:
			Data.RegisterAddress = VCNL36826S_FLAG;
			break;
		case DEV_ID:
			Data.RegisterAddress = DEV_ID;
			break;
		case CALIB_DATA:
			Data.RegisterAddress = CALIB_DATA;
			break;
		default:
			printf("Invalid Register Address\r\n");
			return FAILURE;
		}

		Data.Slave_Address = VCNL36826S_SLAVE_ADD;
		if (ReadI2C_Bus(&Data) != True) {
			printf("I2C Read failed\r\n");
			return FAILURE;
		}

		printf("Register values: 0x%X, 0x%X", Data.RData[0], Data.RData[1]);
	} else {
		printf("Invalid Device Address\r\n");
	}

	return SUCCESS;
}

int i2c_write_t(char *arg1, char *arg2, char *arg3) {
	char *endptr = NULL;
	struct TransferData Data;
	char dev = (char) strtol(arg1, &endptr, 0);
	uint8_t l_byte = 0;
	uint8_t h_byte = 0;
	uint32_t value = 0;

	if (dev == 0x60) {
		char reg = (char) strtol(arg2, &endptr, 0);
		if (*endptr != '\0' || arg2 == endptr) {
			printf("Invalid Register Address\r\n");
			return FAILURE;
		}
		switch (reg) {
		case ST_CONF:
			Data.RegisterAddress = ST_CONF;
			break;
		case PS_CONF1:
			Data.RegisterAddress = PS_CONF1;
			break;
		case PS_CONF3:
			Data.RegisterAddress = PS_CONF3;
			break;
		case PS_THDL:
			Data.RegisterAddress = PS_THDL;
			break;
		case PS_THDH:
			Data.RegisterAddress = PS_THDH;
			break;
		case PS_CANC:
			Data.RegisterAddress = PS_CANC;
			break;
		case PS_AC_LPPERI:
			Data.RegisterAddress = PS_AC_LPPERI;
			break;
		default:
			printf("Invalid Register Address\r\n");
			return FAILURE;
		}

		value = (uint32_t) strtol(arg3, &endptr, 0);
		if ((*endptr != '\0') || arg3 == endptr || value > 0xFFFF) {
			printf("Invalid value\r\n");
			return FAILURE;
		}

		h_byte = (uint8_t) (value & 0xFF);
		l_byte = (uint8_t) ((value >> 8) & 0xFF);
		Data.WData[0] = l_byte;
		Data.WData[1] = h_byte;
		Data.Slave_Address = VCNL36826S_SLAVE_ADD;
		if (WriteI2C_Bus(&Data) != True) {
			printf("I2C Write failed\r\n");
			return FAILURE;
		}

		printf("Register write success\r\n");
	} else {
		printf("Invalid Device Address\r\n");
	}

	return SUCCESS;
}

int get_board_info_t(char *arg1, char *arg2, char *arg3) {
	uint8_t retval = 0;
	system_info_t sys_info = { 0 };

	retval = flash_read_FRU(&sys_info);
	if (retval != SUCCESS) {
		printf("board info-read failed\r\n");
		return FAILURE;
	}
	printf("Board type: %s, Serial number: %s, Magic no.: %s\r\n",
			sys_info.board_type, sys_info.serial_num, sys_info.magic_num);
	return retval;
}

int set_board_info_t(char *arg1, char *arg2, char *arg3) {
	uint8_t retval = 0;
	system_info_t sys_info = { 0 };
	retval = flash_read_FRU(&sys_info);
	strcpy(sys_info.board_type, arg1);
	strcpy(sys_info.serial_num, arg2);
	strcpy(sys_info.magic_num, arg3);

	retval |= Flash_Write_FRU(&sys_info);
	if (retval != SUCCESS)
		return FAILURE;
	return retval;
}
