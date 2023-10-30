#include "flash.h"

#define CRC_ADDRESS		0x08040000
#define MAGIC_NUM_ADDRESS (0x08040000 + 12U)
#define BOARD_TYPE_ADDRESS (0x08040000 + 12U + 4U)
#define SERIAL_NUM_ADDRESS (0x08040000 + 12U + 4U + 20U)
#define CONFIG_ADDRESS (0x08040000 + 12U + 4U + 20U + 20U)

extern bool firstTimeBoot;


/* To write the fru struct to the flash */
uint32_t Flash_Write_FRU_configuration(system_info_t *sys_info) {

	static FLASH_EraseInitTypeDef EraseInitStruct1;
	uint32_t SECTORError;
	printf("Writing FRU configuration data\r\n");

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area */

	/* Get the number of sector to erase from 1st sector */

	/* Fill EraseInit structure*/
//	    EraseInitStruct1.TypeErase     = FLASH_TYPEERASE_SECTORS;
//	    EraseInitStruct1.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
//	    EraseInitStruct1.Sector        = FLASH_SECTOR_2 ; //writing FRU in flash sector 2
//	    EraseInitStruct1.NbSectors     = 1;
	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR | FLASH_FLAG_WRPERR);

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
//	    if (HAL_FLASHEx_Erase(&EraseInitStruct1, &SECTORError) != HAL_OK)
//	    {
//	        return HAL_FLASH_GetError ();
//	    }
	//    printf("\r\nWriting only configuration data into the flash\r\n");
	printf("config [0]: %u\r\n", sys_info->config[0]);
	printf("config [1]: %u\r\n", sys_info->config[1]);
	printf("config [2]: %u\r\n", sys_info->config[2]);
	printf("config [3]: %u\r\n", sys_info->config[3]);
	//	printf("\r\n");
	/* Program the user Flash area word by word
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
//    Flash_Write(CRC_ADDRESS, (uint32_t *)sys_info->crc_arr, 3);
//    Flash_Write(CRC_ADDRESS + (12), (uint32_t *)sys_info->magic_num, 1);
//    Flash_Write(CRC_ADDRESS + (12 + 4), (uint32_t *)sys_info->board_type, 5);
//    Flash_Write(CRC_ADDRESS + (12 + 4 + 20), (uint32_t *)sys_info->serial_num, 5);
	int ret = Flash_Write(CONFIG_ADDRESS, (uint32_t*) sys_info->config[3], 4);
	printf("ret: %d\r\n", ret);
	HAL_FLASH_Lock();
	return SUCCESS;
}

/* To write the fru struct to the flash */
uint32_t Flash_Write_FRU(system_info_t *sys_info) {

	static FLASH_EraseInitTypeDef EraseInitStruct1;
	uint32_t SECTORError;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area */

	/* Get the number of sector to erase from 1st sector */

	/* Fill EraseInit structure*/
	EraseInitStruct1.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct1.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct1.Sector = FLASH_SECTOR_6; //writing FRU in flash sector 2
	EraseInitStruct1.NbSectors = 1;

	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR | FLASH_FLAG_WRPERR);

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	if (HAL_FLASHEx_Erase(&EraseInitStruct1, &SECTORError) != HAL_OK) {
		return HAL_FLASH_GetError();
	}

	//    printf("\r\nWriting data into the flash\r\n");
	//	printf("crc: %u\r\n", sys_info->crc_arr[0]);
	//	printf("image_size: %u\r\n", sys_info->crc_arr[1]);
	//	printf("crc_flag: %u\r\n", sys_info->crc_arr[2]);
	//	printf("config [0]: %u\r\n", sys_info->config[0]);
	//	printf("config [1]: %u\r\n", sys_info->config[1]);
	//	printf("config [2]: %u\r\n", sys_info->config[2]);
	//	printf("config [3]: %u\r\n", sys_info->config[3]);
	//	printf("\r\n");
	//	printf("\r\n");

	/* Program the user Flash area word by word
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Flash_Write(CRC_ADDRESS, (uint32_t*) sys_info->crc_arr, 3);
	Flash_Write(MAGIC_NUM_ADDRESS, (uint32_t*) sys_info->magic_num, 1);
	Flash_Write(BOARD_TYPE_ADDRESS, (uint32_t*) sys_info->board_type, 5);
	Flash_Write(SERIAL_NUM_ADDRESS, (uint32_t*) sys_info->serial_num, 5);
	Flash_Write(CONFIG_ADDRESS, (uint32_t*) sys_info->config, 4);

	HAL_FLASH_Lock();

	return SUCCESS;
}

/* Generic Function to write in flash memory */
uint32_t Flash_CRC_Write(uint32_t StartPageAddress, uint32_t *Data,
		uint16_t numberofwords) {

	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Erase the user Flash area*/

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.NbSectors = 1;
	EraseInitStruct.Sector = FLASH_SECTOR_2;

	__HAL_FLASH_CLEAR_FLAG(
			FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR | FLASH_FLAG_WRPERR);

	if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
		printf("flash erase err\r\n");
		/*Error occurred while page erase.*/
		return HAL_FLASH_GetError();
	}

	/* Program the user Flash area word by word*/

	while (sofar < numberofwords) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress,
				Data[sofar]) == HAL_OK) {
			StartPageAddress += 4; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/
	HAL_FLASH_Lock();

	return SUCCESS;
}

/* To write the config struct to the flash */
uint32_t Flash_Write_config(config_info_t *config_info) {
	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Program the user Flash area word by word
	 (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
	Flash_Write_Data(0x8004100, (uint32_t*) config_info->magic_num, 1);
	Flash_Write_Data(0x800412C, (uint32_t*) config_info->config_ver, 5);
	Flash_Write_Data(0x8004118, (uint32_t*) config_info->backlight_val, 5);
	Flash_Write_Data(0x8004104, (uint32_t*) config_info->crc, 5);

	HAL_FLASH_Lock();
	return SUCCESS;
}

/* Write to the flash memory (for FRU and Config) */
uint32_t Flash_Write_Data(uint32_t StartSectorAddress, uint32_t *Data,
		uint16_t numberofwords) {

	int sofar = 0;
	while (sofar < numberofwords) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress,
				Data[sofar]) == HAL_OK) {
			StartSectorAddress += 4; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}
	/* Lock the Flash to disable the flash control register access (recommended
	 to protect the FLASH memory against possible unwanted operation) *********/

	return SUCCESS;
}
/* To read the data to fru struct */
int flash_read_FRU(system_info_t *sys_info_read) {
	system_info_t sys_info_write;
	memset(sys_info_read->crc_arr, 0, sizeof(sys_info_read->crc_arr));
	memset(sys_info_read->board_type, 0, sizeof(sys_info_read->board_type));
	memset(sys_info_read->serial_num, 0, sizeof(sys_info_read->serial_num));
	memset(sys_info_read->magic_num, 0, sizeof(sys_info_read->magic_num));
	memset(sys_info_read->config, 0, sizeof(sys_info_read->config));
//    memset(sys_info_read->b_version, 0 , sizeof(sys_info_read->b_version));

	Flash_Read_Data(CRC_ADDRESS, (uint32_t*) sys_info_read->crc_arr, 3);
	Flash_Read_Data(MAGIC_NUM_ADDRESS, (uint32_t*) sys_info_read->magic_num, 1);
	Flash_Read_Data(BOARD_TYPE_ADDRESS, (uint32_t*) sys_info_read->board_type, 5);
	Flash_Read_Data(SERIAL_NUM_ADDRESS, (uint32_t*) sys_info_read->serial_num, 5);
	Flash_Read_Data(CONFIG_ADDRESS, (uint32_t*) sys_info_read->config, 4);

#ifdef ENABLE_PRINTS
	printf("sys_info_read.config[0]: %d\r\n", sys_info_read->config[0]);
	printf("sys_info_read.config[1]: %d\r\n", sys_info_read->config[1]);
	printf("sys_info_read.config[2]: %d\r\n", sys_info_read->config[2]);
	printf("sys_info_read.config[3]: %d\r\n", sys_info_read->config[3]);
#endif
	return SUCCESS;
}
/* To read the data to config struct */
int flash_read_config(config_info_t *config_info_read) {
	memset(config_info_read->magic_num, 0, sizeof(config_info_read->magic_num));
	memset(config_info_read->config_ver, 0,
			sizeof(config_info_read->config_ver));
	memset(config_info_read->backlight_val, 0,
			sizeof(config_info_read->backlight_val));
	memset(config_info_read->crc, 0, sizeof(config_info_read->crc));

	Flash_Read_Data(0x08004100, (uint32_t*) config_info_read->magic_num, 1);
	Flash_Read_Data(0x0800412C, (uint32_t*) config_info_read->config_ver, 5);
	Flash_Read_Data(0x08004118, (uint32_t*) config_info_read->backlight_val, 5);
	Flash_Read_Data(0x08004104, (uint32_t*) config_info_read->crc, 5);

	return SUCCESS;
}

/* Generic function to read from data of the flash memory */
void Flash_Read_Data(uint32_t StartSectorAddress, uint32_t *RxBuf,
		uint16_t numberofwords) {
	int count = 0;
	for (count = 0; count < numberofwords; count++) {
		*RxBuf = *(__IO uint32_t*) StartSectorAddress;
		StartSectorAddress += 4;
		RxBuf++;
	}
}

/* Generic Function to write in flash memory */
uint32_t Flash_Write(uint32_t StartPageAddress, uint32_t *Data,
		uint16_t numberofwords) {
	int sofar = 0;
	/* Program the user Flash area word by word*/

	while (sofar < numberofwords) {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartPageAddress,
				Data[sofar]) == HAL_OK) {
			StartPageAddress += 4; // use StartPageAddress += 2 for half word and 8 for double word
			sofar++;
		} else {
			/* Error occurred while writing data in Flash memory*/
			return HAL_FLASH_GetError();
		}
	}

	return SUCCESS;
}
