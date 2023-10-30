/*
 * app_cfg.h
 *
 *  Created on: Aug 19, 2021
 *      Author: vvdn
 */

#ifndef INC_APP_CFG_H_
#define INC_APP_CFG_H_

#include <stdio.h>
#include "cmsis_os.h"

/*
*********************************************************************************************************
*                                           TASK ID
*********************************************************************************************************
*/

#define  APP_STARTUP_TASK_NAME            		"appStartupTask"
#define  WDT_MANAGER_TASK_NAME	                "wdtTask"
#define  AUDIO_MANAGER_TASK_NAME               	"audioManagerTask"
#define  SENSOR_MANAGER_TASK_NAME               "sensorManagerTask"
#define  CLI_MANAGER_TASK_NAME                  "CLI_manager"
#define  IMU_APP_TASK_NAME                      "IMU_App"
#define  COMMAND_APP_TASK_NAME					"command_app"
#define  SYSTEM_MANAGER_TASK_NAME				"systemManagerTask"

/*
*********************************************************************************************************
*                                           TASK NAME
*********************************************************************************************************
*/

#define  APP_STARTUP_TASK_ID              		0x1u
#define  WDT_MANAGER_TASK_ID                    0x2u
#define  AUDIO_MANAGER_TASK_ID                  0x4u
#define  SENSOR_TASK_ID                  		0x8u
#define  CLI_MANAGER_TASK_ID                    0x10u
#define  IMU_APP_TASK_ID                        0x20u
#define  COMMAND_APP_TASK_ID					0x40u

#define  REQUIRED_TASK								SENSOR_TASK_ID | IMU_APP_TASK_ID | COMMAND_APP_TASK_ID

/*
*********************************************************************************************************
*                                          TASK STACK SIZES
*********************************************************************************************************
*/
#define  APP_STARTUP_TASK_STK_SIZE        		128 * 4     //fixed
#define  WDT_MANAGER_TASK_STK_SIZE              128 * 8		//fixed
#define  AUDIO_MANAGER_TASK_STK_SIZE            128 * 4
#define  SENSOR_MANAGER_TASK_STK_SIZE           128 * 16		//fixed
#define  CLI_MANAGER_STK_SIZE                   128 * 16	//fixed
#define  IMU_APP_STK_SIZE                       128 * 8	//fixed
#define  COMMAND_APP_TASK_STK_SIZE				128 * 8

/*
*********************************************************************************************************
*                                           TASK PRIORITIES
*********************************************************************************************************
*/
#define  APP_STARTUP_TASK_PRIO     				osPriorityHigh7
#define  WDT_MANAGER_TASK_PRIO          		osPriorityHigh7
#define  AUDIO_MANAGER_TASK_PRIO              	osPriorityNormal
#define  SENSOR_MANAGER_TASK_PRIO         		osPriorityRealtime
#define  CLI_MANAGER_TASK_PRIO                  osPriorityHigh7
#define  IMU_TASK_TASK_PRIO                     osPriorityRealtime
#define  COMMAND_APP_TASK_PRIO					osPriorityHigh7

#define WDT_FEED_INTERVAL 						1

#endif /* INC_APP_CFG_H_ */
