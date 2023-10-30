/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define APP_CRC_ADDRESS			0x08008000			// This is where the CRC is stored
#define APP_SIZE_ADDRESS		0x08008004			// This is where the image_size is stored
#define CRC_FLAG_ADDRESS		0x08008008			// This is where the CRC flag is stored
#define HUB_DEV_ADDRESS			0x5A
#define BOOTLOADER_VERSION		(char *)"1.000"

typedef  void (*pFunction)(void);
#define APPLICATION_ADDRESS   (uint32_t)0x0800C000
pFunction Jump_To_Application;
uint32_t JumpAddress;
/* USER CODE END PTD */

/* Structure contains fru data */
typedef struct system_info {
    unsigned char magic_num[4];
    unsigned char board_type[20];
    unsigned char serial_num[20];
    unsigned char b_version[8];
}system_info_t;

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
I2C_HandleTypeDef hi2c3;
UART_HandleTypeDef huart2;
//#define ENABLE_CRC_CALCULATION   //uncomment this to calculate crc
/* USER CODE BEGIN PV */
uint32_t image_crc = 0;
uint32_t image_size = 0;
system_info_t sys_info = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
static void JUMp(void);

uint32_t Flash_Write(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords);
void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords);
uint32_t Flash_CRC_Write(uint32_t StartPageAddress, uint32_t *Data,uint16_t numberofwords);
uint32_t Flash_Write_FRU (system_info_t *sys_info);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t retval = 0 ;
	uint32_t CRCcalSize = 0;
	uint32_t flag = 0;
	uint8_t debugBuf[100] = {0};
	/* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
   MX_GPIO_Init();
   MX_CRC_Init();
   MX_I2C3_Init();
   MX_USART2_UART_Init();

#ifndef ENABLE_CRC_CALCULATION
//   flash_read_FRU(&sys_info);
//   if(strcmp(sys_info.b_version, BOOTLOADER_VERSION) != 0){
//	   strcpy(sys_info.b_version, BOOTLOADER_VERSION);
//	   Flash_Write_FRU(&sys_info);
//   }
   /* USER CODE BEGIN 2 */

   Flash_Read_Data(APP_SIZE_ADDRESS, &image_size, 1);
   Flash_Read_Data(APP_CRC_ADDRESS, &image_crc, 1);
   Flash_Read_Data(CRC_FLAG_ADDRESS, &flag, 1);
   if(flag == 1){
	   JUMp();  // already CRC checked jump to application
   }
   //FIXME: Remove the below statement once R&D is done
   else {
	   JUMp();
   }
#else
   //FIXME: Change this image size once the new firmware is ready for calculating the new CRC
   image_size = 146024; //change it whenever calculating CRC for new firmware
#endif
   CRCcalSize = image_size / 4;
   if(image_size % 4 != 0)
	   CRCcalSize += 1;
   retval = HAL_CRC_Calculate(&hcrc, (uint32_t *)APPLICATION_ADDRESS, CRCcalSize);
#ifdef ENABLE_CRC_CALCULATION
   sprintf(debugBuf, "CRC %u\r\n", retval);
      HAL_UART_Transmit(&huart2, debugBuf, strlen(debugBuf), 1000);
#else
   if(retval == image_crc){
	  uint32_t buf[3] = {0};
	  buf[0] = image_crc;
	  buf[1] = image_size;
	  buf[2] = (uint32_t)1;
	  Flash_CRC_Write((uint32_t)APP_CRC_ADDRESS, buf, (uint16_t)3);
 	  JUMp();
   }
   else {
 	 /*Should Reset here and go into DFU mode*/
      sprintf(debugBuf, "calculater CRC %u flash crc %u err\r\n", retval, image_crc);
	  HAL_UART_Transmit(&huart2, debugBuf, strlen(debugBuf), 1000);
	  hub_init();
 	  HAL_CRC_DeInit(&hcrc);
 	  HAL_UART_DeInit(&huart2);
 	  HAL_I2C_DeInit(&hi2c3);
 	  NVIC_SystemReset();

   }
//   JUMp();
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void hub_init(void)
{
	uint8_t buf[10] = {0};

	/* gpio 9 direction */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x05;	//No. of bytes to write in memory
	buf[3] = 0x00;
	buf[4] = 0x01;	//No. of bytes to write
	buf[5] = 0x08;	//GPIO DIRECTION register
	buf[6] = 0x32;
	buf[7] = 0x02;

	if(HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS , (uint8_t *)buf, 8, 100)){
		printf("Direction set failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if(HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS , (uint8_t *)buf, 3, 100)){
		printf("Direction set command execute failed\r\n");
	}
	/* gpio 9 value */

	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x05;
	buf[3] = 0x00;
	buf[4] = 0x01;
	buf[5] = 0x08;
	buf[6] = 0x36;
	buf[7] = 0x02;	//setting as 1

	if(HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS , (uint8_t *)buf, 8, 100)){
		printf("GPIO set failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if(HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS , (uint8_t *)buf, 3, 100)){
		printf("GPIO set command execute failed\r\n");
	}

	/* attach command with smb AA56 */
	buf[0] = 0xAA;
	buf[1] = 0x56;
	buf[2] = 0x00;
	if(HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS , (uint8_t *)buf, 3, 100)){
		printf("attach failed\r\n");
	}
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

static void JUMp(void)
{
	/* Test if user code is programmed starting from address "APPLICATION_ADDRESS" */
	if (((*(__IO uint32_t*)APPLICATION_ADDRESS) & 0x2FFE0000 ) == 0x20000000)
	{
		HAL_UART_DeInit(&huart2);
		HAL_I2C_DeInit(&hi2c3);
		HAL_CRC_DeInit(&hcrc);
		HAL_RCC_DeInit();
		HAL_DeInit();

		SysTick->CTRL = 0;
		SysTick->LOAD = 0;
		SysTick->VAL = 0;

		__disable_irq(); //Note: Make sure to enable_irq in the secondary image

		SCB->VTOR = APPLICATION_ADDRESS;
	 /* Jump to user application */
	  JumpAddress = *(__IO uint32_t*) (APPLICATION_ADDRESS + 4);
	  Jump_To_Application = (pFunction) JumpAddress;
	 /* Initialize user application's Stack Pointer */
	  __set_MSP(*(__IO uint32_t*) APPLICATION_ADDRESS);
	  Jump_To_Application();
	}
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

/* To write the fru struct to the flash */
uint32_t Flash_Write_FRU (system_info_t *sys_info)
{

    static FLASH_EraseInitTypeDef EraseInitStruct1;
    uint32_t SECTORError;

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Erase the user Flash area */

    /* Get the number of sector to erase from 1st sector */

    /* Fill EraseInit structure*/
    EraseInitStruct1.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct1.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct1.Sector        = FLASH_SECTOR_1 ; //writing FRU in flash sector 1
    EraseInitStruct1.NbSectors     = 1;

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
    		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR |
			FLASH_FLAG_WRPERR);

    /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
       you have to make sure that these data are rewritten before they are accessed during code
       execution. If this cannot be done safely, it is recommended to flush the caches by setting the
       DCRST and ICRST bits in the FLASH_CR register. */
    if (HAL_FLASHEx_Erase(&EraseInitStruct1, &SECTORError) != HAL_OK)
    {
        return HAL_FLASH_GetError ();
    }

    /* Program the user Flash area word by word
       (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/
    Flash_Write(0x8004000, (uint32_t *)sys_info->magic_num, 1);
    Flash_Write(0x8004004, (uint32_t *)sys_info->board_type, 5);
    Flash_Write(0x8004018, (uint32_t *)sys_info->serial_num, 5);
    Flash_Write(0x800402C, (uint32_t *)sys_info->b_version, 2);

    HAL_FLASH_Lock();
    return SUCCESS;
}


uint32_t Flash_Write(uint32_t StartSectorAddress, uint32_t *Data, uint16_t numberofwords)
{

    int sofar=0;
    while (sofar<numberofwords)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartSectorAddress, Data[sofar]) == HAL_OK)
        {
            StartSectorAddress += 4;  // use StartPageAddress += 2 for half word and 8 for double word
            sofar++;
        }
        else
        {
            /* Error occurred while writing data in Flash memory*/
            return HAL_FLASH_GetError ();
        }
    }

    return SUCCESS;
}

/* To read from data of the flash memory */
void Flash_Read_Data (uint32_t StartSectorAddress, uint32_t *RxBuf, uint16_t numberofwords)
{
	int count = 0;
	for(count = 0; count < numberofwords; count++)
    {
        *RxBuf = *(__IO uint32_t *)StartSectorAddress;
        StartSectorAddress += 4;
        RxBuf++;
    }
}

/* To read the data to fru struct */
int flash_read_FRU(system_info_t *sys_info_read)
{
    memset(sys_info_read->board_type, 0 , sizeof(sys_info_read->board_type));
    memset(sys_info_read->serial_num, 0 , sizeof(sys_info_read->serial_num));
    memset(sys_info_read->magic_num, 0 , sizeof(sys_info_read->magic_num));
    memset(sys_info_read->b_version, 0 , sizeof(sys_info_read->b_version));

    Flash_Read_Data(0x8004000, (uint32_t *)sys_info_read->magic_num, 1);
    Flash_Read_Data(0x8004004, (uint32_t *)sys_info_read->board_type, 5);
    Flash_Read_Data(0x8004018, (uint32_t *)sys_info_read->serial_num, 5);
    Flash_Read_Data(0x800402C, (uint32_t *)&sys_info_read->b_version, 2);

    return SUCCESS;
}

static void MX_I2C3_Init(void)
{

    /* USER CODE BEGIN I2C3_Init 0 */

    /* USER CODE END I2C3_Init 0 */

    /* USER CODE BEGIN I2C3_Init 1 */

    /* USER CODE END I2C3_Init 1 */
    hi2c3.Instance = I2C3;
    hi2c3.Init.ClockSpeed = 100000;
    hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c3.Init.OwnAddress1 = 0;
    hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c3.Init.OwnAddress2 = 0;
    hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C3_Init 2 */

    /* USER CODE END I2C3_Init 2 */
}

/* Generic Function to write in flash memory */
uint32_t Flash_CRC_Write(uint32_t StartPageAddress, uint32_t *Data,uint16_t numberofwords)
{

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

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_OPERR |
    		FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR | FLASH_FLAG_RDERR |
			FLASH_FLAG_WRPERR);

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {
    	printf("flas erase err\r\n");
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


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
