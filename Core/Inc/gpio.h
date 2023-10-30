#ifndef __GPIO_H
#define __GPIO_H

#include "stdint.h"
#include "main.h"

/* GPIO pull resistors */
#define PULLUP            GPIO_PULLUP
#define PULLDOWN          GPIO_PULLDOWN
#define NOPULL            GPIO_NOPULL

/* GPIO speed */
#define SPEED_LOW         GPIO_SPEED_FREQ_LOW
#define SPEED_MEDIUM      GPIO_SPEED_FREQ_MEDIUM
#define SPEED_HIGH        GPIO_SPEED_FREQ_HIGH
#define SPEED_VERY_HIGH   GPIO_SPEED_FREQ_VERY_HIGH

/* To set gpio as input 
 * param: GPIOx - GPIO_port(GPIOA / GPIOB / GPIOC)
 *        GPIO_PIN_X - GPIO pin number (0 to 15)
 *        pull - PULLUP,PULLDOWN,NOPULL*/
void gpio_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_X,uint32_t pull);

/* To set gpio as output 
 * param: GPIOx - GPIO_port(GPIOA / GPIOB / GPIOC)
 *        GPIO_PIN_X - GPIO pin number (0 to 15)
 *        pull - GPIO_PULLUP,GPIO_PULLDOWN,GPIO_NOPULL
 *        speed - SPEED_LOW, SPEED_HIGH, SPEED_MEDIUM, SPEED_VERY_HIGH*/
void gpio_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_X, uint32_t pull, uint32_t speed);

/* To reset the gpio 
 * param: GPIOx - GPIO_port(GPIOA / GPIOB / GPIOC) 
 *        GPIO_PIN_X - GPIO pin number (0 to 15)*/
void reset_gpio_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_X);

/* To set the gpio 
 * param: GPIOx - GPIO_port(GPIOA / GPIOB / GPIOC)
 *        GPIO_PIN_X - GPIO pin number (0 to 15)*/
void set_gpio_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_X);

#endif // __GPIO_H
