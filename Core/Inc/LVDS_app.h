/*
 * display_cfg.h
 *
 *  Created on: Aug 24, 2021
 *      Author: vvdn
 */

#ifndef INC_LVDS_APP_H_
#define INC_LVDS_APP_H_

#include <stdio.h>
#include <stdbool.h>

#define LT7211B_ADDR		0x56
#define LT_RESET_PORT	GPIOB
#define LT_RESET_PIN	GPIO_PIN_0
#define LT_GPIO1_PORT	GPIOA
#define LT_GPIO1_PIN		GPIO_PIN_15

int8_t LVDS_FWU(void);
void resetLT7211B_v(bool resetFlag);
void turnOffDisplayLT7211B(void);

#endif /* INC_LVDS_APP_H_ */
