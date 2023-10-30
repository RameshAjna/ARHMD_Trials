/*
 * I2C_Functions.c
 *
 * Created  : 8 June 2020
 * Modified : 5 November 2020
 * Author   : HWanyusof
 */ 
 
#include "I2C_Functions.h"

extern I2C_HandleTypeDef hi2c3;

extern SemaphoreHandle_t proxMutex;

uint8_t WriteI2C_Byte(struct TransferData *Data)
{
	int Error = 0;
	uint8_t WData = Data->WData[0];
	Error = HAL_I2C_Master_Transmit(&hi2c3,(Data->Slave_Address)<<1, &WData,1,100);
	if (Error !=HAL_OK){
		return -1;
	}else {
		return HAL_OK;
	}
}

uint8_t WriteI2C_Bus(struct TransferData *Data)
{
	int Error = 0;
//	while (hi2c3.State != HAL_I2C_STATE_READY)
//	{
//		/* wait until i2c became ready */
//	}
//	uint8_t WData[3]={Data->RegisterAddress, Data->WData[0], Data->WData[1]};
//	Error = HAL_I2C_Master_Transmit(&hi2c3, (Data->Slave_Address)<<1, WData, 3, 100);
	Error = BlockingWrite(Data);
	if (Error !=HAL_OK){
		return -1;
	}else {
		return HAL_OK;
	}
}

uint8_t ReadI2C_Bus (struct TransferData *Data)
{
	int Error= 0;
//	while (hi2c3.State != HAL_I2C_STATE_READY)
//	{
//		/* wait until i2c became ready */
//	}
//	Error = HAL_I2C_Mem_Read(&hi2c3, (Data->Slave_Address)<<1, Data->RegisterAddress, 1,Data->RData,2,100);
	Error = BlockingRead(Data);
	if (Error !=HAL_OK){
		printf("read err: %d\r\n", Error);
		return -1;
	}else {
		return HAL_OK;
	}
}

uint8_t BlockingRead (struct TransferData *Data)
{
	int ret = 0;
	xSemaphoreTake(proxMutex, portMAX_DELAY);

	ret = HAL_I2C_Mem_Read(&hi2c3, (Data->Slave_Address)<<1, Data->RegisterAddress, 1,Data->RData,2,100);

    xSemaphoreGive(proxMutex);
    return ret;
}

uint8_t BlockingWrite (struct TransferData *Data)
{
	int ret = 0;
	xSemaphoreTake(proxMutex, portMAX_DELAY);

	uint8_t WData[3]={Data->RegisterAddress, Data->WData[0], Data->WData[1]};
	ret = HAL_I2C_Master_Transmit(&hi2c3, (Data->Slave_Address)<<1, WData, 3, 100);

    xSemaphoreGive(proxMutex);
    return ret;
}
