#ifndef __COMMANDS_H
#define __COMMANDS_H

#include "main.h"
#include "prox.h"
#include "flash.h"

#define MAX_NUM_CMDS 28

#define FAILURE 1

#define HUB_DEV_ADDRESS	0x5A

/*commands structure definition*/
typedef struct cli_command {
    char *cmd_name;
    int  no_of_arg;
    int (*cmd_handler) (char *arg1,char *arg2,char *arg3);
}CLI_CMD_T;

/*
 * CLI Manager functions
 */
int reboot_t(char *arg1, char *arg2,char *arg3);
int imu_read_data_t(char *arg1, char *arg2,char *arg3);
int imu_enable_t(char *arg1, char *arg2,char *arg3);
int enable_acc_t(char *arg1, char *arg2,char *arg3);
int enable_gyro_t(char *arg1, char *arg2,char *arg3);
int enable_mag_t(char *arg1, char *arg2,char *arg3);
int enable_RV_t(char *arg1, char *arg2,char *arg3);
int imu_calibrate_t(char *arg1, char *arg2, char *arg3);
int imu_cal_save_t(char *arg1, char *arg2, char *arg3);
int prox_enable_t(char *arg1, char *arg2,char *arg3);
int prox_set_TH_t(char *arg1, char *arg2,char *arg3);
int prox_get_TH_t(char *arg1, char *arg2,char *arg3);
int prox_get_data_t(char *arg1, char *arg2,char *arg3);
int set_backlight_t(char *arg1, char *arg2,char *arg3);
int get_backlight_t(char *arg1, char *arg2,char *arg3);
int trigger_sys_upgrade_t(char *arg1, char *arg2,char *arg3);
int rtc_set_t(char *arg1, char *arg2,char *arg3);
int rtc_get_t(char *arg1, char *arg2,char *arg3);
int gpio_read_pin_t(char *arg1, char *arg2,char *arg3);
int gpio_write_pin_t(char *arg1, char *arg2, char *arg3);
int version_read_t(char *arg1, char *arg2, char *arg3);
int set_time_out_t(char *arg1, char *arg2,char *arg3);
int set_format_t(char *arg1, char *arg2,char *arg3);
int i2c_read_t(char *arg1, char *arg2,char *arg3);
int i2c_write_t(char *arg1, char *arg2,char *arg3);
int set_board_info_t(char *arg1, char *arg2,char *arg3);
int get_board_info_t(char *arg1, char *arg2,char *arg3);
int set_luminous_t(char *arg1, char *arg2,char *arg3);
int get_luminous_t(char *arg1, char *arg2,char *arg3);

/* command structure */
extern const CLI_CMD_T clicommands[MAX_NUM_CMDS];
//CLI_CMD_T *cmd_list;

/*support functions*/
void disable_sensors(void);

#endif //  __COMMANDS_H

