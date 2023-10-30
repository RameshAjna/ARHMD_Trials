/*
 * display_cfg.c
 *
 *  Created on: Aug 24, 2021
 *      Author: vvdn
 */

#include <LVDS_app.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "stream_buffer.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;
extern StreamBufferHandle_t xStreamBuffer;
extern USBD_HandleTypeDef usbdHandle;
int dataRcvd = 0;

//char debugBuffer[50];

long LTFwSize;
//extern uint8_t LTFwSizeRcvd;
uint8_t WriteData[16];
int keyDataLen = 286;
uint8_t keyData[286];

static int8_t LVDS_Config(void);
static int8_t LVDS_Block_Erase(void);
static int8_t LVDS_Get_Chip_ID(int16_t *chipID);
static int8_t LVDS_read_key(void);
static int8_t LVDS_WriteI2C_Byte(int8_t addr, int8_t data);
static uint8_t LVDS_ReadI2C_Byte(int8_t addr);
static int8_t LVDS_WriteI2C_ByteN(int8_t addr, int8_t *data, int8_t len);
static uint8_t LVDS_ReadI2C_ByteN(uint8_t addr, uint8_t *data, uint8_t len);
void LVDS_Write();
void LVDS_Read( );
void LVDS_Write_Key();

osSemaphoreId_t i2c_rxSem;
const osSemaphoreAttr_t i2c_rx_sem_attributes = {
  .name = "i2c_rx_sem"
};

void clearDisplayLT7211B_v(void) {
	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(DISP_1_8V_GPIO_PORT, DISP_1_8V_GPIO_PIN, GPIO_PIN_RESET);
	osDelay(500);
	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN, GPIO_PIN_SET);
	osDelay(100);
}

void resetLT7211B_v(bool resetFlag) {
	if (resetFlag) reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
	else set_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
}

static void LVDS_i2c_Reset(void) {
    HAL_I2C_DeInit(&hi2c1);
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    HAL_I2C_Init(&hi2c1);
}

int8_t LVDS_FWU(void)
{
	int16_t chipID = 0;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	reset_gpio_pin(GPIOC, GPIO_PIN_0);
	printf("imu disabled\r\n");

	LVDS_i2c_Reset();
	printf("i2c reset\r\n");
	//i2c_rxSem = osSemaphoreNew(1, 1, &i2c_rx_sem_attributes);
	if( (LVDS_Get_Chip_ID(&chipID) < 0) || (chipID != 0x1605) )
	{
		printf("LVDS invalid chip id %d \r\n", chipID);
		return -1;
	}
	osDelay(1000);

	HAL_GPIO_DeInit(LT_GPIO1_PORT, LT_GPIO1_PIN);
    if( LVDS_Config() < 0)
    {
    	printf("LVDS config failure\r\n");
    	return -1;
    }
    osDelay(1000);

    if( LVDS_read_key() < 0)
    {
        printf("LVDS read_key failure\r\n");
        return -1;
    }

    LVDS_Block_Erase();
    osDelay(1000);
#if 1
    LVDS_Write();

//    GPIO_InitStruct.Pin = LT_GPIO1_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(LT_GPIO1_PORT, &GPIO_InitStruct);
//    HAL_GPIO_WritePin(LT_GPIO1_PORT, LT_GPIO1_PIN,  GPIO_PIN_SET);
    /* reboot lontium */
    osDelay(1000);
    //HAL_GPIO_WritePin(LT_RESET_PORT, LT_RESET_PIN,  GPIO_PIN_RESET);
//    osDelay(200);
//    HAL_GPIO_WritePin(LT_RESET_PORT, LT_RESET_PIN,  GPIO_PIN_SET);
//    LVDS_Read();

//    LVDS_Write_Key();
#endif
    return 0;
}

static int8_t LVDS_WriteI2C_Byte(int8_t addr, int8_t data)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(&hi2c1, LT7211B_ADDR, addr, 1, (uint8_t *)&data, 1, 1000);
	if ( ret != HAL_OK )
	{
		printf("write i2c err %d \r\n", ret);
		return -1;
	}
	return 0;

}

static int8_t LVDS_WriteI2C_ByteN(int8_t addr, int8_t *data, int8_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(&hi2c1, LT7211B_ADDR, addr, 1, (uint8_t *)data, len, 100);
	if ( ret != HAL_OK )
	{
		printf("write i2c N err %d \r\n", ret);
		return -1;
	}
	return 0;

}

static uint8_t LVDS_ReadI2C_ByteN(uint8_t addr, uint8_t *data, uint8_t len)
{
	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Read(&hi2c1, LT7211B_ADDR, addr, 1, data, len, 100);
	if ( ret != HAL_OK )
	{
		printf("Read i2c N err %d \r\n", ret);
		return 0;
	}
	return 1;
}
static uint8_t LVDS_ReadI2C_Byte(int8_t addr)
{
	HAL_StatusTypeDef ret;
	uint8_t data;
	dataRcvd = 0;
	ret = HAL_I2C_Mem_Read(&hi2c1, LT7211B_ADDR, addr, 1, &data, 1, 100);
	if ( ret != HAL_OK )
	{
		printf("Read i2c err %d \r\n", ret);
		return -1;
	}
	return data;
}
//static uint8_t LVDS_ReadI2C_Byte_IT(int8_t addr)
//{
//	HAL_StatusTypeDef ret;
//	uint8_t data;
//	dataRcvd = 0;
//	ret = HAL_I2C_Mem_Read_IT(&hi2c1, LT7211B_ADDR, addr, 1, &data, 1);
//	if ( ret != HAL_OK )
//	{
//		printf("Read i2c err %d \r\n", ret);
//		return -1;
//	}
//	printf("waiting rx cb \r\n");
//	while(!dataRcvd);
//	return data;
//}

//void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
//{
//	printf("mstr rx cb \r\n");
//	//osSemaphoreRelease(i2c_rxSem);
//	dataRcvd = 1;
//}


static int8_t LVDS_Get_Chip_ID(int16_t *chipID)
{
	HAL_StatusTypeDef ret;
	uint8_t buf[2];
	uint8_t data[2] = {0,0};

	ret = HAL_I2C_IsDeviceReady(&hi2c1, LT7211B_ADDR, 1, 100);
	if ( ret != HAL_OK )
	{
		printf("dev rdy err %d \r\n", ret);
		return -1;
	}

	buf[0] = 0xFF;
	buf[1] = 0x80;
	ret = LVDS_WriteI2C_Byte(buf[0], buf[1]);

	buf[0] = 0xEE;
	buf[1] = 0x01;
	ret = LVDS_WriteI2C_Byte(buf[0], buf[1]);
	//ret = HAL_I2C_Mem_Write(&hi2c1, LT7211B_ADDR, buf[0], 1, &buf[1], 1, 100);
	//if ( ret != HAL_OK )
//	{
//		sprintf(debugBuffer, "get chp id 2 err %d \r\n", ret);
//		HAL_UART_Transmit(&huart2, (uint8_t *)debugBuffer, strlen(debugBuffer), 100);
//		return -1;
//	}

	buf[0] = 0xFF;
	buf[1] = 0xA0;
	ret = LVDS_WriteI2C_Byte(buf[0], buf[1]);
//	ret = HAL_I2C_Mem_Write(&hi2c1, LT7211B_ADDR, buf[0], 1, &buf[1], 1, 100);
//	if ( ret != HAL_OK )
//	{
//		sprintf(debugBuffer, "get chp id 3 err %d \r\n", ret);
//		HAL_UART_Transmit(&huart2, (uint8_t *)debugBuffer, strlen(debugBuffer), 100);
//		return -1;
//	}


	buf[0] = 0x00;
	buf[1] = 0x01;
	data[0] = LVDS_ReadI2C_Byte(buf[0]);
//	ret = HAL_I2C_Mem_Read(&hi2c1, LT7211B_ADDR, buf[0], 1, &data[0], 1, 100);
//	if ( ret != HAL_OK )
//	{
//		sprintf(debugBuffer, "get chp id rd 1 err %d \r\n", ret);
//		HAL_UART_Transmit(&huart2, (uint8_t *)debugBuffer, strlen(debugBuffer), 100);
//		return -1;
//	}
    data[1] = LVDS_ReadI2C_Byte(buf[1]);
//	ret = HAL_I2C_Mem_Read(&hi2c1, LT7211B_ADDR, buf[1], 1, &data[1], 1, 100);
//	if ( ret != HAL_OK )
//	{
//		sprintf(debugBuffer, "get chp id rd 1 err %d \r\n", ret);
//		HAL_UART_Transmit(&huart2, (uint8_t *)debugBuffer, strlen(debugBuffer), 100);
//		return -1;
//	}

	*chipID = ((data[0] << 8) | data[1]);

//	printf("get chp id %x %x %ld \r\n", data[0], data[1], *chipID);
	return 0;
}

static int8_t LVDS_Config(void)
{
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0xEE, 0x01 );

	LVDS_WriteI2C_Byte( 0x5A, 0x82 );
	LVDS_WriteI2C_Byte( 0x5E, 0xC0 );
	LVDS_WriteI2C_Byte( 0x58, 0x00 );
	LVDS_WriteI2C_Byte( 0x59, 0x51 );
	LVDS_WriteI2C_Byte( 0x5A, 0x92 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	return 0;
}

static int8_t LVDS_read_key(void)
{
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0xEE, 0x01 );

		//¶ÁÈ¡Ö®Ç°reset fifo
	LVDS_WriteI2C_Byte( 0xFF, 0x90 );
	uint8_t bRead = LVDS_ReadI2C_Byte( 0x02 );
	bRead &= 0xDF;
	LVDS_WriteI2C_Byte( 0x02, bRead );
	bRead |= 0x20;
	LVDS_WriteI2C_Byte( 0x02, bRead );

		//wren enable
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0x5A, 0x86 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	long lReadAddr = 0x6000;//KeyŽæŽ¢ÔÚFlashÀïµÄµØÖ·ÊÇ0x6000
	int8_t addr[3] = { 0, 0, 0 };
	addr[0] = ( lReadAddr & 0xFF0000 ) >> 16;
	addr[1] = ( lReadAddr & 0xFF00 )   >>  8;
	addr[2] = lReadAddr & 0xFF;

	int nPage = 286/ 16;//Key×Ü¹²286žö×ÖœÚ£¬ÅäÖÃÒ»ŽÎ×î¶à¶ÁÐŽ16žö×ÖœÚ
	if ( 286 % 16 != 0 )
	{
		++nPage;
	}
	memset( keyData, 0, 256 );
	for ( int i=0; i<nPage; ++i )//Ã¿ŽÎ×î¶à¶Á16žö×ÖœÚ£¬×Ü¹²¶ÁnpageŽÎ
	{
		//ÉèÖÃ¶ÁÈ¡µØÖ·
		LVDS_WriteI2C_Byte( 0x5E, 0x60 | 0xF );
		LVDS_WriteI2C_Byte( 0x5A, 0xA2 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );
		LVDS_WriteI2C_Byte( 0x5B, addr[0] );
		LVDS_WriteI2C_Byte( 0x5C, addr[1] );
		LVDS_WriteI2C_Byte( 0x5D, addr[2] );
		LVDS_WriteI2C_Byte( 0x5A, 0x92 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );
		LVDS_WriteI2C_Byte( 0x58, 0x01 );

		uint8_t nPageReadLen = 16;
		if ( 286 - i * 16 < 16)//×îºóÒ»ŽÎ³€¶ÈÃ»ÓÐ16žö£¬ŒÆËã×îºóÒ»ŽÎµÄÊýŸÝ³€¶È
		{
			nPageReadLen = 286 - i * 16;//nPageReadLenÊÇ×îºóÒ»ŽÎµÄ³€¶È
		}

			//œ«¶ÁµœµÄKeyÊýŸÝŽ¢ŽæÆðÀŽ

		uint8_t bRet = LVDS_ReadI2C_ByteN( 0x5F, keyData + (i * nPageReadLen), nPageReadLen );//ŽÓ0x5FÖÐ¶ÁÈ¡

			//ÏÂŽÎ¶ÁÈ¡Ê±µØÖ·ÔöŒÓ16žö
		lReadAddr += nPageReadLen;
		addr[0] = ( lReadAddr & 0xFF0000 ) >> 16;
		addr[1] = ( lReadAddr & 0xFF00 )   >>  8;
		addr[2] = lReadAddr & 0xFF;
	}

	return 0;
}

static int8_t LVDS_Block_Erase(void)
{
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0xEE, 0x01 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	LVDS_WriteI2C_Byte( 0x5A, 0x86 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	LVDS_WriteI2C_Byte( 0x5B, 0x00 );
	LVDS_WriteI2C_Byte( 0x5C, 0x00 );
	LVDS_WriteI2C_Byte( 0x5D, 0x00 );

	LVDS_WriteI2C_Byte( 0x5A, 0x83 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	return 0;

}

void LVDS_Write()
{
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0xEE, 0x01 );

	//ÐŽÖ®Ç°reset fifo
	LVDS_WriteI2C_Byte( 0xFF, 0x90 );
	uint8_t bRead = LVDS_ReadI2C_Byte( 0x02 );
	bRead &= 0xDF;
	LVDS_WriteI2C_Byte( 0x02, bRead );
	bRead |= 0x20;
	LVDS_WriteI2C_Byte( 0x02, bRead );

	//wren enable
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0x5A, 0x86 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	//ÉèÖÃDataÊýŸÝµÄÆðÊŒµØÖ·
	uint8_t addr[3] = { 0, 0, 0 };
	long lWriteAddr = 0;
	long lStartAddr = lWriteAddr;
	uint8_t byWriteData[16];
//	LTFwSize = 16480;
	int nPage = LTFwSize/ 16;//DatalenÊÇ¹ÌŒþÊýŸÝµÄ×ÖœÚÊý£¬ÅäÖÃÒ»ŽÎ×î¶à¶ÁÐŽ16žö×ÖœÚ
	if ( LTFwSize % 16 != 0 )
	{
		++nPage;
	}
#if 1

	//sending acknowledgement
	ack_packet_t ack_packet = {.ack_id = FW_UP_REQ, .ack_status = ACK_SUCCESS, .ack_data[0] = 0, .ack_data[1] = 0};
	USBD_Command_Transmit(&usbdHandle, &ack_packet, sizeof(ack_packet));

	printf("waiting for data \r\n");
	while(xStreamBufferBytesAvailable(xStreamBuffer) == 0)
	{
	}
//	while(1){}
//	printf("data filled \r\n");
#endif
	static long overall_rcvd = 0;
	printf("receiving data from usb\r\n");
	for ( int j = 0; j < nPage; ++j )
	{
		wdt_keepalive(COMMAND_APP_TASK_ID);
		// ÐŽwrenÃüÁî(ÎªÒ»žöpulse£¬ŽËÊ±²»ÐèÒª¿ŒÂÇwrrd_mode£¬spi_paddr[1:0]µÄÖµ)
		LVDS_WriteI2C_Byte( 0x5A, 0x86 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );

		// ÅäÖÃspi_len[3:0]= 15£¬¿ÉÅäÖÃ£¬spiÄÚ²¿ŒÓ1£¬ŒŽÅäÖÃÒ»ŽÎÐŽÈë16žö×ÖœÚ
		LVDS_WriteI2C_Byte( 0x5E, 0xE0 | 0xF );
		LVDS_WriteI2C_Byte( 0x5A, 0xA2 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );

		LVDS_WriteI2C_Byte( 0x58, 0x01 );
        int rcvd = 0;

        while(rcvd == 0)
        {
        	rcvd = xStreamBufferReceive(xStreamBuffer, byWriteData, sizeof(byWriteData), 0);
        }
        overall_rcvd += rcvd;



		LVDS_WriteI2C_ByteN( 0x59, byWriteData, 16 );//byWriteDataÖžÏò¹ÌŒþÊýŸÝµÄÆðÊŒµØÖ·£¬Ã¿ŽÎÐŽ16žö£¬×îºó²»×ã16žöµÄ²¹FF

		// °ÑfifoÊýŸÝÐŽµœflash(µ±wrrd_mode = 1£¬spi_paddr= 2¡¯b10,addr[23:0](µØÖ·ÔÚÐŽÈë¹ý³ÌÖÐÐèÒª±£³Ö²»±ä)×Œ±žºÃµÄÇé¿öÏÂ£¬žøÒ»žöspi_staµÄpulse£¬ŸÍ¿ªÊŒÐŽÈëflash)
		LVDS_WriteI2C_Byte( 0x5B, addr[0] );
		LVDS_WriteI2C_Byte( 0x5C, addr[1] );
		LVDS_WriteI2C_Byte( 0x5D, addr[2] );
		LVDS_WriteI2C_Byte( 0x5E, 0xE0 );
		LVDS_WriteI2C_Byte( 0x5A, 0x92 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );

		lStartAddr += 16;
		addr[0] = ( lStartAddr & 0xFF0000 ) >> 16;
		addr[1] = ( lStartAddr & 0xFF00 ) >> 8;
		addr[2] =   lStartAddr & 0xFF;
//		printf("rcvd %d %x\r\n", overall_rcvd, byWriteData[0]);

		if((overall_rcvd % 80 == 0) && (overall_rcvd != LTFwSize))
		{
//			printf("rcv %d\r\n", overall_rcvd);
			ack_packet_t ack_packet = {.ack_id = FW_UP_REQ, .ack_status = ACK_SUCCESS, .ack_data[0] = 0, .ack_data[1] = 0};
			USBD_Command_Transmit(&usbdHandle, &ack_packet, sizeof(ack_packet));
		}

	}
	LVDS_WriteI2C_Byte( 0x5A, 0x8A );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );
	printf("done %d\r\n", overall_rcvd);

}

void LVDS_Read( )
{
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0xEE, 0x01 );

	//¶ÁÖ®Ç°reset fifo
	LVDS_WriteI2C_Byte( 0xFF, 0x90 );
	uint8_t bRead = LVDS_ReadI2C_Byte( 0x02 );
	bRead &= 0xDF;
	LVDS_WriteI2C_Byte( 0x02, bRead );
	bRead |= 0x20;
	LVDS_WriteI2C_Byte( 0x02, bRead );

	//wren enable
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0x5A, 0x86 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	long lReadAddr = 0;//¶ÁÊýŸÝµÄÆðÊŒµØÖ·ÊÇ00
    uint8_t addr[3] = { 0, 0, 0 };
	addr[0] = ( lReadAddr & 0xFF0000 ) >> 16;
	addr[1] = ( lReadAddr & 0xFF00 )   >>  8;
	addr[2] = lReadAddr & 0xFF;

	int WriteDataLen; //ÐŽÈëÊ±µÄÊýŸÝ³€¶È

	int nPage = WriteDataLen / 16;//DatalenÊÇ¹ÌŒþÊýŸÝµÄ×ÖœÚÊý£¬ÅäÖÃÒ»ŽÎ×î¶à¶ÁÐŽ16žö×ÖœÚ
	if ( WriteDataLen % 16 != 0 )
	{
		++nPage;
	}

	for ( int i=0; i<nPage; ++i )
	{
		LVDS_WriteI2C_Byte( 0x5E, 0x60 | 0xF );
		LVDS_WriteI2C_Byte( 0x5A, 0xA2 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );
		LVDS_WriteI2C_Byte( 0x5B, addr[0] );
		LVDS_WriteI2C_Byte( 0x5C, addr[1] );
		LVDS_WriteI2C_Byte( 0x5D, addr[2] );
		LVDS_WriteI2C_Byte( 0x5A, 0x92 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );
		LVDS_WriteI2C_Byte( 0x58, 0x01 );

		uint8_t nPageReadLen = 16;
		if ( WriteDataLen - i * 16 < 16 )//×îºóÒ»ŽÎ³€¶ÈÃ»ÓÐ16žö£¬ŒÆËã×îºóÒ»ŽÎµÄÊýŸÝ³€¶È
		{
			nPageReadLen = WriteDataLen - i * 16;
		}
		//œ«¶Á³öÀŽµÄÊýŸÝºÍ¹ÌŒþÊýŸÝ±ÈœÏÒ»ÏÂ
		uint8_t byPageReadData[128];
		memset( byPageReadData, 0, 128 );
		uint8_t bRet = LVDS_ReadI2C_ByteN( 0x5F, byPageReadData, nPageReadLen );

		//ÏÂŽÎ¶ÁÈ¡Ê±µØÖ·ÔöŒÓ16žö
		lReadAddr += nPageReadLen;
		addr[0] = ( lReadAddr & 0xFF0000 ) >> 16;
		addr[1] = ( lReadAddr & 0xFF00 )   >>  8;
		addr[2] = lReadAddr & 0xFF;
	}
}


void LVDS_Write_Key()
{
	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0xEE, 0x01 );

	//ÐŽÖ®Ç°reset fifo
	LVDS_WriteI2C_Byte( 0xFF, 0x90 );
	int8_t bRead = LVDS_ReadI2C_Byte( 0x02 );
	bRead &= 0xDF;
	LVDS_WriteI2C_Byte( 0x02, bRead );
	bRead |= 0x20;
	LVDS_WriteI2C_Byte( 0x02, bRead );


	int8_t addr[3] = { 0x00, 0x60, 0x00 };

	LVDS_WriteI2C_Byte( 0xFF, 0x80 );
	LVDS_WriteI2C_Byte( 0x5A, 0x86 );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );

	//¶ÁKeyµÄÆðÊŒµØÖ·ÔÚ0x6000
	long lWriteAddr = 0x6000;
	long lStartAddr = lWriteAddr;
	long lEndAddr   = lWriteAddr;

	int nPage = 286/ 16;//286ÊÇHCP KeyµÄ×ÖœÚÊý£¬ÅäÖÃÒ»ŽÎ×î¶à¶ÁÐŽ16žö×ÖœÚ
	if ( keyDataLen % 16 != 0 )
	{
		++nPage;
	}

	for ( int j = 0; j < nPage; ++j )
	{
		// ÐŽwrenÃüÁî(ÎªÒ»žöpulse£¬ŽËÊ±²»ÐèÒª¿ŒÂÇwrrd_mode£¬spi_paddr[1:0]µÄÖµ)
		LVDS_WriteI2C_Byte( 0x5A, 0x86 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );

		// ÅäÖÃspi_len[3:0]= 15£¬¿ÉÅäÖÃ£¬spiÄÚ²¿ŒÓ1£¬ŒŽÅäÖÃÒ»ŽÎÐŽÈë16žö×ÖœÚ
		LVDS_WriteI2C_Byte( 0x5E, 0xE0 | 0xF );
		LVDS_WriteI2C_Byte( 0x5A, 0xA2 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );

		LVDS_WriteI2C_Byte( 0x58, 0x01 );

		LVDS_WriteI2C_ByteN( 0x59, keyData+j*16, 16 );//byWriteDataÖžÏòKeyÊýŸÝµÄÆðÊŒµØÖ·£¬Ã¿ŽÎÐŽ16žö£¬×îºó²»×ã16žöµÄ²¹FF

		// °ÑfifoÊýŸÝÐŽµœflash(µ±wrrd_mode = 1£¬spi_paddr= 2¡¯b10,addr[23:0](µØÖ·ÔÚÐŽÈë¹ý³ÌÖÐÐèÒª±£³Ö²»±ä)×Œ±žºÃµÄÇé¿öÏÂ£¬žøÒ»žöspi_staµÄpulse£¬ŸÍ¿ªÊŒÐŽÈëflash)
		LVDS_WriteI2C_Byte( 0x5B, addr[0] );
		LVDS_WriteI2C_Byte( 0x5C, addr[1] );
		LVDS_WriteI2C_Byte( 0x5D, addr[2] );
		LVDS_WriteI2C_Byte( 0x5E, 0xE0 );
		LVDS_WriteI2C_Byte( 0x5A, 0x92 );
		LVDS_WriteI2C_Byte( 0x5A, 0x82 );

		lStartAddr += 16;
		addr[0] = ( lStartAddr & 0xFF0000 ) >> 16;
		addr[1] = ( lStartAddr & 0xFF00 ) >> 8;
		addr[2] =   lStartAddr & 0xFF;
	}
	// ÐŽwrdiœûÖ¹ÃüÁî(ÎªÒ»žöpulse£¬ŽËÊ±²»ÐèÒª¿ŒÂÇwrrd_mode£¬spi_paddr[1:0]µÄÖµ)
	LVDS_WriteI2C_Byte( 0x5A, 0x8A );
	LVDS_WriteI2C_Byte( 0x5A, 0x82 );
//	printf("key written\r\n");

}

void I2C1_clock_factor(int factor)
{

    /* USER CODE BEGIN I2C1_Init 0 */
    //
    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */
    //
    /* USER CODE END I2C1_Init 1 */
	hi2c1.Init.ClockSpeed = factor*100000;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */
    //
    /* USER CODE END I2C1_Init 2 */

}

