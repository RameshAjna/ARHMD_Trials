/*
 * I2C_Functions.h
 *
 * Created  : 8 June 2020
 * Modified : 5 November 2020
 * Author   : HWanyusof
 */ 
 
#pragma once 

#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"


struct TransferData
{
	uint8_t RegisterAddress;
	uint8_t WData[2];
	uint8_t RData[2];
	uint8_t length;
	uint8_t Slave_Address;
	uint8_t Select_I2C_Bus;
};


struct GestureTransferData
{
	uint8_t RegisterAddress;
	uint8_t WData[2];
	uint8_t RData[6];
	uint8_t length;
	uint8_t Slave_Address;
	uint8_t Select_I2C_Bus;
};
 
uint8_t WriteI2C_Byte(struct TransferData *Data);
uint8_t ReadI2C_Bus(struct TransferData *Data);
uint8_t WriteI2C_Bus(struct TransferData *Data);

/*Support APIs*/
uint8_t BlockingRead (struct TransferData *Data);
uint8_t BlockingWrite (struct TransferData *Data);

