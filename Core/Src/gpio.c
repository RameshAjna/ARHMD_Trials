#include "gpio.h"

void gpio_input(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_X,uint32_t pull)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_X;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = pull;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void gpio_output(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_X, uint32_t pull, uint32_t speed)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOx, GPIO_PIN_X, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_X;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = speed;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void gpio_interrupt(GPIO_TypeDef *GPIOx, uint16_t GPIO_PIN_X, uint32_t pull, uint32_t mode)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    HAL_GPIO_WritePin(GPIOx, GPIO_PIN_X, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = GPIO_PIN_X;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = pull;
    HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);

}

/* To reset the gpio */
void reset_gpio_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_X)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_PIN_X, GPIO_PIN_RESET);
}

/* To set the gpio */
void set_gpio_pin(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_X)
{
    HAL_GPIO_WritePin(GPIOx, GPIO_PIN_X, GPIO_PIN_SET);
}

/* To set the gpio */
GPIO_PinState get_gpio_pin_status(GPIO_TypeDef* GPIOx, uint16_t GPIO_PIN_X)
{
	return HAL_GPIO_ReadPin(GPIOx, GPIO_PIN_X);
}
