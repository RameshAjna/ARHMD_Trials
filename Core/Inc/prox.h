/*
 * prox.h
 *
 *  Created on: 22-Sep-2021
 *      Author: vvdn
 */

#ifndef INC_PROX_H_
#define INC_PROX_H_

#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "sensor_proximity.h"
#include "I2C_Functions.h"
#define PROX_DUMMY_DATA	5000

typedef enum {
	OK = 0,
	FAIL = 1,
}states_t;

#define PROX_CONVERTER 	0.049		//200/4096(maximum distance/maximum ADC value)

/* Starting the Proximity Sensor */
uint8_t start_Prox_Sensor(void);

/*Stopping the Proximity Sensor*/
uint8_t stop_Prox_Sensor(void);

/*Enabling wakeup pin*/
uint8_t enable_Prox_int(void);

/*Disabling wakeup pin*/
uint8_t disable_prox_int(void);

/*Set thresholds*/
uint8_t set_TH(uint16_t L_threshold, uint16_t H_threshold);

#endif /* INC_PROX_H_ */
