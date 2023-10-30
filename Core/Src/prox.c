/*
 * prox.c
 *
 *  Created on: 22-Sep-2021
 *      Author: vvdn
 */

#include "prox.h"

//Initial values for threshold
uint16_t low_th = 80;
uint16_t high_th = 90;
int prox_enabled = 0;

extern I2C_HandleTypeDef hi2c3;

uint8_t start_Prox_Sensor(void){
	uint8_t ret = 0;
	if(prox_enabled)
	{
		return (uint8_t)OK;
	}
	HAL_Delay(50);
	ret = HAL_I2C_IsDeviceReady(&hi2c3, 0x60<<1, 100, 100);
	if (ret != HAL_OK){
		printf("prox i2c not ready \r\n");
		//Unable to detect device
		return (uint8_t)FAIL;
	}

	ret = prox_open();
	if(ret != True){
		printf("prox start error\r\n");
		//Unable to open device
		return (uint8_t)FAIL;
	}

	prox_enabled = 1;
	return (uint8_t)OK;
}

uint8_t stop_Prox_Sensor(void){
	uint8_t ret = 0;

	if(!prox_enabled)
	{
		return (uint8_t)OK;
	}
	ret = prox_close();
	if(ret != True){
		//Unable to close device
		return (uint8_t)FAIL;
	}

	prox_enabled = 0;
	return (uint8_t)OK;
}

uint8_t enable_Prox_int(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/*Enabling GPIO*/
	GPIO_InitStruct.Pin = GPIO_PIN_0;
  	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  	GPIO_InitStruct.Pull = GPIO_PULLUP;
  	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  	/* EXTI interrupt init*/
  	HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 1);
  	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	return (uint8_t)OK;
}

uint8_t disable_prox_int(void)
{

	HAL_NVIC_DisableIRQ(EXTI0_IRQn);
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
	printf("prox interrupt disabled\r\n");
	return (uint8_t)OK;
}

uint8_t set_TH(uint16_t L_threshold, uint16_t H_threshold)
{
	if(L_threshold > H_threshold){
		return (uint8_t)FAIL;
	}
	if(SET_PS_LowThreshold(L_threshold)!= True){
		return (uint8_t)FAIL;
	}
	if(SET_PS_HighThreshold(H_threshold) != True){
		return (uint8_t)FAIL;
	}
	return (uint8_t)OK;
}

uint16_t prox_read(void)
{
	if(prox_enabled)
	{
		return prox_get_data();
	}
	else
	{
		return PROX_DUMMY_DATA;
	}
}
