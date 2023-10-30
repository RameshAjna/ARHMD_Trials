/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASH_H
#define __FLASH_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "main.h"

#define BACKL		0
#define SLEEP_TIM	3

/* Structure contains fru data */
typedef struct system_info {
	uint32_t crc_arr[3];
    unsigned char magic_num[4];
    unsigned char board_type[20];
    unsigned char serial_num[20];
    uint32_t config[4];
    unsigned char b_version[8];
}system_info_t;

/* Structure contains configuration data */
typedef struct config_info {
	unsigned char magic_num[4];
	unsigned char crc[20];
	unsigned char config_ver[20];
	unsigned char backlight_val[20];
}config_info_t;

/* To write the fru struct to the flash */
uint32_t Flash_Write_FRU (system_info_t *sys_info);

uint32_t Flash_Write_FRU_configuration(system_info_t *sys_info);

/* To write the config struct to the flash */
uint32_t Flash_Write_config (config_info_t *config_info);
/* Write to the flash memory (for FRU and Config)*/
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);
/* To read the data to fru struct */
int flash_read_FRU(system_info_t *sys_info_read);
/* To read the data to config struct */
int flash_read_config(config_info_t *config_info_read);
/* To read from data of the flash memory */
void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *Rx, uint16_t numberofwords);
/* Write to the flash memory */
uint32_t Flash_Write(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);
/* Write to the flash memory */
uint32_t Flash_CRC_Write(uint32_t StartPageAddress, uint32_t *Data,uint16_t numberofwords);

#endif  /* __FLASH_H */
