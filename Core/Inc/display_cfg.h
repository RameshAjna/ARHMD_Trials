/*
 * display_cfg.h
 *
 *  Created on: Sep 1, 2021
 *      Author: vvdn
 */

#ifndef INC_DISPLAY_CFG_H_
#define INC_DISPLAY_CFG_H_

#define R_DISP_NSS_PIN			GPIO_PIN_4
#define L_DISP_NSS_PIN			GPIO_PIN_12
#define R_DISP_NSS_PORT			GPIOA
#define L_DISP_NSS_PORT			GPIOB
#define DISP_XCLR_GPIO_PIN		GPIO_PIN_8
#define DISP_1_8V_GPIO_PIN		GPIO_PIN_7
#define DISP_XCLR_GPIO_PORT		GPIOC
#define DISP_1_8V_GPIO_PORT		GPIOC
#define SPI_TIMEOUT				100			/* blocking 100 ms */
/**
  * @brief  SPI handle Structure definition
  */
typedef enum Display_Format
{
	Format_1 = 0,			/* 60/59.94 Hz */
	Format_2,				/* 50 Hz */
	Format_3				/* 47.952/48 Hz */
} Display_Format;

#endif /* INC_DISPLAY_CFG_H_ */
