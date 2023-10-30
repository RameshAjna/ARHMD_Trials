#include <stdio.h>
#include "display_cfg.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "flash.h"

void Display_Configuration(Display_Format format);

extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
char debugBuffer[50];
int rd_flag = 0;
int wr_flag = 0;
system_info_t sys_info = { 0 };

uint8_t brightness = 255;
uint8_t lumin = 0;

/* from pdf */
//uint8_t disp_init_data[130]={
//0X0E,0X00,0X40,0XA0,0X5F,0X80,0X00,0X40,0X00,0X56,0X00,0X00,0X00,0X00,0X00,0X00,
//0X00,0X00,0X00,0X00,0X00,0XC0,0X40,0X40,0X80,0X40,0X40,0X40,0X0A,0X00,0X22,0X10,
//0X60,0X44,0X20,0X29,0X61,0X00,0X00,0X00,0X40,0X58,0X28,0X00,0X00,0X00,0X19,0X1A,
//0X44,0X0D,0X0E,0XFF,0X00,0X24,0XDD,0X00,0X05,0X04,0X35,0X07,0X5D,0X0A,0XE6,0X03,
//0X5D,0X00,0X01,0X7B,0X00,0X17,0X76,0X00,0X76,0X00,0X5A,0X00,0X0A,0X5D,0X0A,0X5D,
//0X00,0X0A,0X41,0X00,0X51,0X0A,0X38,0X40,0X62,0X2F,0X00,0X00,0X00,0X00,0X00,0X00,
//0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X00,0X04,0XE8,0X03,
//0X00,0X4E,0X4E,0X00,0X41,0X27,0X85,0X30,0X06,0X36,0X80,0X25,0X47,0X61,0X00,0X00,
//0X01,0X81,
//};
/* fromexcel */
//uint8_t disp_init_data[130]={
//0x0E, 0x00, 0x40, 0xA0, 0x5F, 0x80, 0x00, 0x40, 0x00, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x40, 0x40, 0x80, 0x40, 0x40, 0x40, 0x0A, 0x5D, 0x22, 0x10,
//0x60, 0x44, 0x20, 0x29, 0x61, 0x00, 0x00, 0x00, 0x40, 0x58, 0x28, 0x00, 0x00, 0x00, 0x19, 0x1A,
//0x44, 0x0D, 0x0E, 0xFF, 0x00, 0x24, 0xDD, 0x00, 0x05, 0x04, 0x35, 0x07, 0x5D, 0x0A, 0xE6, 0x03,
//0x5D, 0x00, 0x01, 0x7B, 0x00, 0x17, 0x76, 0x00, 0x76, 0x00, 0x5A, 0x00, 0x0A, 0x5D, 0x0A, 0x5D,
//0x00, 0x0A, 0x41, 0x00, 0x51, 0x0A, 0x38, 0x40, 0x62, 0x2F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xE8, 0x03,
//0x00, 0x4E, 0x4E, 0x00, 0x41, 0x27, 0x85, 0x30, 0x06, 0x36, 0x80, 0x25, 0x47, 0x61, 0x00, 0x00,
//0x01, 0x81};
uint8_t disp_init_data[130] = { 0x0E, 0x00, 0x40, 0xA0, 0x5F, 0x80, 0x00, 0x40,
		0x00, 0x56, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xC0, 0x40, 0x40, 0x80, 0x40, 0x40, 0x40, 0x0A, 0x5D, 0x22, 0x10,
		0x4E, 0x44, 0x0E, 0x29, 0x61, 0x00, 0x00, 0x00, 0x40, 0x46, 0x16, 0x00,
		0x00, 0x00, 0x19, 0x1A, 0x44, 0x0D, 0x0E, 0xFF, 0x00, 0x24, 0xDD, 0x00,
		0x05, 0x04, 0x35, 0x07, 0x5D, 0x0A, 0xE6, 0x03, 0x5D, 0x00, 0x01, 0x7B,
		0x00, 0x17, 0x76, 0x00, 0x76, 0x00, 0x5A, 0x00, 0x0A, 0x5D, 0x0A, 0x5D,
		0x00, 0x0A, 0x41, 0x00, 0x51, 0x0A, 0x38, 0x40, 0x62, 0x2F, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0xE8, 0x03, 0x00, 0x4E, 0x4E, 0x00,
		0x41, 0x27, 0x85, 0x30, 0x06, 0x36, 0x80, 0x25, 0x47, 0x61, 0x00, 0x00,
		0x01, 0x81 };
/* from board*/
uint8_t initial_disp_data[130] = { 0X0E, 0X00, 0X00, 0X00, 0X1F, 0x00, 0X00,
		0X00, 0X00, 0X56, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00,
		0X00, 0X00, 0XC0, 0X40, 0X40, 0X80, 0X40, 0X40, 0X40, 0X10, 0X80, 0X40,
		0X10, 0X60, 0X44, 0X20, 0X29, 0X61, 0X00, 0X04, 0X4C, 0X40, 0X58, 0X28,
		0X04, 0X65, 0X00, 0X18, 0X19, 0X44, 0X0D, 0X0E, 0X80, 0X00, 0X24, 0XDD,
		0X00, 0X01, 0X04, 0X35, 0X07, 0X5D, 0X0A, 0XB6, 0X03, 0X8D, 0X00, 0X01,
		0X7B, 0X00, 0X17, 0X76, 0X00, 0X76, 0X00, 0X5A, 0X00, 0X0A, 0X5D, 0X0A,
		0X5D, 0X00, 0X0A, 0X41, 0X00, 0X51, 0X0A, 0X38, 0X40, 0X62, 0X2F, 0X00,
		0X76, 0X00, 0X01, 0X0B, 0X00, 0X01, 0XA0, 0X00, 0X02, 0X0F, 0X00, 0X00,
		0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0XE8, 0X00, 0X00, 0X00, 0X00,
		0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X30, 0X00,
		0X00, 0X01, 0XFF };

/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Right Panel Write                                                                         */
/* Parameter:                                                                                              */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Write the data by the SPI                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void SPI_R_PANEL_Write(uint8_t data) {
	HAL_SPI_Transmit(&hspi1, &data, 1, SPI_TIMEOUT);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Left Panel Write                                                                          */
/* Parameter:                                                                                              */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Write the data by the SPI                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void SPI_L_PANEL_Write(uint8_t data) {
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Right Panel Write Register                                                                */
/* Parameter:    addr:Register address; data:The data of Register                                          */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Write the data to the Register                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SPI_R_PANEL_Write_Reg(uint8_t addr, uint8_t data) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	SPI_R_PANEL_Write(addr);
	SPI_R_PANEL_Write(data);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Left Panel Write Register                                                                 */
/* Parameter:    addr:Register address; data:The data of Register                                          */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Write the data to the Register                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void SPI_L_PANEL_Write_Reg(uint8_t addr, uint8_t data) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	SPI_L_PANEL_Write(addr);
	SPI_L_PANEL_Write(data);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Right Panel Write Multiple Register                                                       */
/* Parameter:    addr:Register address; *data:The poniter of Register's data;n:The number of data;         */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Write the data to the Multiple Register                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SPI_R_PANEL_Write_Multi_Reg(uint8_t addr, uint8_t *data, uint32_t n) {
	uint8_t i;

	/* assuming NSS will be low */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	SPI_R_PANEL_Write(addr);
	for (i = 0; i < n; i++) {
		SPI_R_PANEL_Write(*data);
		data++;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	/* need to check whether nss remaining low affects spi */
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Left Panel Write Multiple Register                                                        */
/* Parameter:    addr:Register address; *data:The poniter of Register's data;n:The number of data;         */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Write the data to the Multiple Register                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SPI_L_PANEL_Write_Multi_Reg(uint8_t addr, uint8_t *data, uint32_t n) {
	uint8_t i;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	SPI_L_PANEL_Write(addr);
	for (i = 0; i < n; i++) {
		SPI_R_PANEL_Write(*data);
		data++;
	}
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Right Panel Read                                                                          */
/* Parameter:                                                                                              */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Read the data from the panel                                                              */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t R_SPI_Read() {
	uint8_t data;
	HAL_SPI_Receive(&hspi1, &data, 1, SPI_TIMEOUT);
	return data;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Left Panel Read                                                                           */
/* Parameter:                                                                                              */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Read the data from the panel                                                              */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t L_SPI_Read() {
	uint8_t data;
	return data;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Right Panel Read the Register                                                             */
/* Parameter:                                                                                              */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Read the data from the Register                                                           */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t R_SPI_Reg_Read(uint8_t addr) {
	uint8_t data;
	uint8_t w_addr = 0x81;
	SPI_R_PANEL_Write_Reg(w_addr, addr);
	osDelay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi1, &data, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	return data;
}
/*---------------------------------------------------------------------------------------------------------*/
/* Function:                                                                                               */
/*               Left Panel Read the Register                                                              */
/* Parameter:                                                                                              */
/* Returns:                                                                                                */
/* Description:                                                                                            */
/*               Read the data from the Register                                                           */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t L_SPI_Reg_Read(uint8_t addr) {
	uint8_t data;
	uint8_t w_addr = 0x81;
	SPI_L_PANEL_Write_Reg(w_addr, addr);
	osDelay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, SPI_TIMEOUT);
	HAL_SPI_Receive(&hspi2, &data, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	return data;
}

void Display_Set_Brightness(int arg) {
	brightness = (uint8_t) arg;

	SPI_L_PANEL_Write_Reg(0x18, (uint8_t) brightness);
	SPI_R_PANEL_Write_Reg(0x18, (uint8_t) brightness);
	printf("rcv_dat L %x R %x\r\n", L_SPI_Reg_Read(0x18), R_SPI_Reg_Read(0x18));
}

void Display_Get_Brightness(int *arg) {
	*arg = brightness;
}

//Control Luminance of the display panel
void Display_Set_Luminous(int arg) {
	lumin = (uint8_t) arg;

	SPI_L_PANEL_Write_Reg(0x1D, (uint8_t) arg);
	SPI_R_PANEL_Write_Reg(0x1D, (uint8_t) arg);
	printf("rcv_dat L %x R %x\r\n", L_SPI_Reg_Read(0x1D), R_SPI_Reg_Read(0x1D));
}

void Display_Get_Luminous(int *arg) {
	*arg = lumin;
}

void Display_Configuration(Display_Format format) {
	uint8_t recv_dat = 0;
	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN, GPIO_PIN_RESET);
	osDelay(100);
	HAL_GPIO_WritePin(DISP_1_8V_GPIO_PORT, DISP_1_8V_GPIO_PIN, GPIO_PIN_SET);
	printf("1.8v on\r\n");
	osDelay(500);
	printf("dly ovr\r\n");

	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN, GPIO_PIN_SET);
	osDelay(150);

	for (int count = 0; count < 130; count++) {
		SPI_L_PANEL_Write_Reg(count, disp_init_data[count]);
		SPI_R_PANEL_Write_Reg(count, disp_init_data[count]);
	}

//	SPI_L_PANEL_Write_Reg(0x80,0x01);
//	printf("L data 02 %x 03 %x 04 %x 6d %x 6f %x 71 %x 72 %x\r\n", L_SPI_Reg_Read(0x02), \
//			L_SPI_Reg_Read(0x03), L_SPI_Reg_Read(0x04), L_SPI_Reg_Read(0x6D), L_SPI_Reg_Read(0x6F), \
//			L_SPI_Reg_Read(0x71), L_SPI_Reg_Read(0x72));
//	printf("R data 02 %x 03 %x 04 %x 6d %x 6f %x 71 %x 72 %x\r\n", R_SPI_Reg_Read(0x02), \
//			R_SPI_Reg_Read(0x03), R_SPI_Reg_Read(0x04), R_SPI_Reg_Read(0x6D), R_SPI_Reg_Read(0x6F), \
//			R_SPI_Reg_Read(0x71), R_SPI_Reg_Read(0x72));

	SPI_L_PANEL_Write_Reg(0x00, 0x0F);
	SPI_R_PANEL_Write_Reg(0x00, 0x0F);
	osDelay(100);

	SPI_L_PANEL_Write_Reg(0x01, 0x01);                //3
	SPI_R_PANEL_Write_Reg(0x01, 0x01);
	osDelay(500);
	printf("10v on\r\n");
	osDelay(500);
	printf("dly ovr\r\n");

	SPI_L_PANEL_Write_Reg(0x04, 0x3f);                //4
	SPI_R_PANEL_Write_Reg(0x04, 0x3f);
	osDelay(100);

	SPI_L_PANEL_Write_Reg(0x71, 0x46);                //5
	SPI_L_PANEL_Write_Reg(0x72, 0x46);
	SPI_R_PANEL_Write_Reg(0x71, 0x46);
	SPI_R_PANEL_Write_Reg(0x72, 0x46);
	osDelay(100);

	SPI_L_PANEL_Write_Reg(0x6D, 0x00);                //6
	SPI_L_PANEL_Write_Reg(0x6F, 0x00);
	SPI_L_PANEL_Write_Reg(0x71, 0x00);
	SPI_L_PANEL_Write_Reg(0x72, 0x00);
	SPI_R_PANEL_Write_Reg(0x6D, 0x00);
	SPI_R_PANEL_Write_Reg(0x6F, 0x00);
	SPI_R_PANEL_Write_Reg(0x71, 0x00);
	SPI_R_PANEL_Write_Reg(0x72, 0x00);

	SPI_L_PANEL_Write_Reg(0x03, 0x20);                //7
	SPI_R_PANEL_Write_Reg(0x03, 0x20);

//	printf("rcv_dat L %x R %x\r\n", L_SPI_Reg_Read(0x18), R_SPI_Reg_Read(0x18));
	osDelay(100);

	SPI_L_PANEL_Write_Reg(0x08, 0x05);          // to change brightness
	SPI_R_PANEL_Write_Reg(0x08, 0x05);

	int value = 0;
	flash_read_FRU(&sys_info);
	value = (int) sys_info.config[BACKL];
	printf("value: %d\r\n", value);

	Display_Set_Brightness(value);
}

void Display_Configuration_2(Display_Format format) {
//	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN, GPIO_PIN_RESET);
//		osDelay(1);
//	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_1_8V_GPIO_PIN, GPIO_PIN_SET);
//	osDelay(1); 			/* sleep for 500 us */
//
//	HAL_GPIO_WritePin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN, GPIO_PIN_SET);
//	osDelay(1);
#if 0
	uint8_t data1 = 0, rcv_dat = 0xFF;
	data1 = 0x02;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x40;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x03;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0xA0;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x04;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x5F;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x6D;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x04;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x6F;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x03;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x71;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x4E;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x72;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x4E;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);

	data1 = 0x00;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x0F;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);

	data1 = 0x01;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x01;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);

	data1 = 0x04;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x3F;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);

	data1 = 0x71;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x46;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x72;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x46;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);

	data1 = 0x6D;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x6F;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x71;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x72;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);

	data1 = 0x03;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x20;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);

	data1 = 0x80;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x01;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);
	data1 = 0x08;
		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
		data1 = 0x01;
		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
		osDelay(1);
		data1 = 0x18;
			HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
			data1 = 0x1E;
			HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
			osDelay(1000);
#if 0
while(1)
{
	printf("f brt\r\n");

	data1 = 0x18;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0xFE;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1000);

	printf("0 brt\r\n");
	data1 = 0x18;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	data1 = 0x00;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1000);
}
#endif
while(1)
{
	data1 = 0x81;
	//data1 = 0x2E;
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//	data1 = 0x18;
//	//data1 = 0x30;
//	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);
//
//	data1 = 0x81;
//	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//	data1 = 0x00;
//		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//	osDelay(1);
//	while(rcv_dat == 0xFF)
//	{
//	HAL_SPI_Receive(&hspi2, &rcv_dat, 1, SPI_TIMEOUT);
//	osDelay(1);
//	}
//	printf("data %x\r\n", rcv_dat);
//	osDelay(1);
//	HAL_SPI_Receive(&hspi2, &data1, 1, SPI_TIMEOUT);
//	printf("data1 %x\r\n", data1);
//
//	osDelay(1);
}
#else
	uint8_t addr = 0, data1 = 0, rcv_dat = 0xFFFF;
//#if 0
#if 0
	data1 = 0x4002;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0xA003;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x5F04;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x046D;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x036F;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x4E71;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x4E72;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
#endif
#if 0
	data1 = 0xF000;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);

	data1 = 0x0101;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);

	data1 = 0x3F04;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);

	data1 = 0x4671;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x4672;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);

	data1 = 0x006D;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x006F;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x0071;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	data1 = 0x0072;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

	data1 = 0x2003;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);
#endif
	addr = 0x80;
	data1 = 0x01;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, SPI_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);

	//data1 = 0xA018;
	addr = 0x18;
	data1 = 0xAF;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &addr, 1, SPI_TIMEOUT);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);
	while (1) {
		addr = 0x81;
		data1 = 0x18;

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &addr, 1, SPI_TIMEOUT);
		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		osDelay(1);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi2, &addr, 1, SPI_TIMEOUT);
		HAL_SPI_Receive(&hspi2, &rcv_dat, 1, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		osDelay(1);

		printf("rcv_dat %lx\r\n", rcv_dat);
	}
#if 0
	data1 = 0xFF18;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	osDelay(1);

	//data1 = 0x1008;
		while(1)
		{
		osDelay(1);
		data1 = 0x8103;
		data1 = 0x1881;
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		//					//data1 = 0x18A0;
		//			HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		//					osDelay(1);
		//data1 = 0x8103;
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, &data1, &rcv_dat, 1, SPI_TIMEOUT);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		osDelay(1);
		printf("rcv_dat %lx\r\n", rcv_dat);
		}
#endif
//#endif
#if 0
	//data1 = 0x8011;
			//while(1)
			{
	data1 = 0x8001;

	HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
	osDelay(1);
	data1 = 0x0801;

		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
		osDelay(1);
			}

	while(1)
	{
		osDelay(1);
		data1 = 0x8136;
		//data1 = 0x18A0;
		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
		osDelay(1);
		data1 = 0x8136;
		HAL_SPI_TransmitReceive(&hspi2, &data1, &rcv_dat, 1, SPI_TIMEOUT);
		osDelay(1);
		sprintf(debugBuffer, "rcv_dat %lx\r\n", rcv_dat);
	  if(HAL_UART_Transmit(&huart2, debugBuffer, strlen(debugBuffer), 100) != HAL_OK)
	  {
			  /* error */
	  }
	}
		//HAL_StatusTypeDef sts = 0;
		//data1 = 0x8180;
//		data1 = 0x8081;
//		HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//
//		HAL_SPI_Receive(&hspi2, &rcv_dat, 1, SPI_TIMEOUT);
//		sprintf(debugBuffer, "rcv_dat %lx\r\n", rcv_dat);
//			  if(HAL_UART_Transmit(&huart2, debugBuffer, strlen(debugBuffer), 100) != HAL_OK)
//			  {
//					  /* error */
//			  }
				//HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//		//sts = HAL_SPI_Transmit_IT(&hspi2, &data1, 1);
//
				osDelay(1);
//				data1 = 0x8180;
//				while(1)
//				{
////				data1 = 0x0818;
//				HAL_SPI_Receive(&hspi2, &data1, 1, SPI_TIMEOUT);
//				sprintf(debugBuffer, "rcv_dat %lx\r\n", rcv_dat);
//					  if(HAL_UART_Transmit(&huart2, debugBuffer, strlen(debugBuffer), 100) != HAL_OK)
//					  {
//							  /* error */
//					  }
//					  osDelay(1);
//				}
//////								HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//////								osDelay(1);
//////												HAL_SPI_Receive(&hspi2, &data1, 1, SPI_TIMEOUT);
////												osDelay(1);
//////		data1 = 0x8118;
//////				HAL_SPI_Transmit(&hspi2, &data1, 1, SPI_TIMEOUT);
//				osDelay(1);
	//}
#endif
#endif

}

void delayTimer(uint32_t time) {
	uint32_t uwTimclock = 0;
	uint32_t uwPrescalerValue = 0;
	uint32_t pFLatency;
	TIM_HandleTypeDef htim2;
	/*Configure the TIM2 IRQ priority */
	HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);

	/* Enable the TIM2 global Interrupt */
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable TIM2 clock */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* Compute TIM2 clock */
	uwTimclock = 2 * HAL_RCC_GetPCLK1Freq();
	/* Compute the prescaler value to have TIM2 counter clock equal to 1MHz */
	uwPrescalerValue = (uint32_t) ((uwTimclock / 1000000U) - 1U);

	/* Initialize TIM2 */
	htim2.Instance = TIM2;

	/* Initialize TIMx peripheral as follow:
	 + Period = [(TIM2CLK/1000) - 1]. to have a (1/1000) s time base.
	 + Prescaler = (uwTimclock/1000000 - 1) to have a 1MHz counter clock.
	 + ClockDivision = 0
	 + Counter direction = Up
	 */
	htim2.Init.Period = 10000000 - 1; //(1000000U / 1000000U) - 1U;
	htim2.Init.Prescaler = uwPrescalerValue;
	htim2.Init.ClockDivision = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;

	if (HAL_TIM_Base_Init(&htim2) == HAL_OK) {
		/* Start the TIM time Base generation in interrupt mode */
		HAL_TIM_Base_Start_IT(&htim2);
	}
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	printf("er cb\r\n");
	wr_flag = 1;
	rd_flag = 1;
}

