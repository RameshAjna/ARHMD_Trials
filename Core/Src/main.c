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
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "sh2.h"
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// ARHMD DEMO FW : Activated Vectors
// RV Vector as RV_*
// GRV Vector as AR_RV_*

#define SOFTWARE_VERSION_MAJOR		1
#define SOFTWARE_VERSION_MINOR		1
#define SOFTWARE_VERSION_PATCH		9

#define ENABLE_PRINTS
#define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))

extern void IMU_Init(I2C_HandleTypeDef *i2c);
// Number of sensor events that can be queued before dropping data.
// A good value would be twice the number of sensors enabled by the app.
#define SENSOR_EVENT_QUEUE_SIZE 	(40)
//#define ENABLE_SLEEP_MODE 			1
#define DISABLE_AUDIO 				1

/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;
system_info_t sys_info_write;
system_info_t sys_info_read;
config_info_t config_info;
config_info_t config_info_read;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi1;
int enable_iwdg_refresh = 0;

StreamBufferHandle_t xStreamBuffer;
USBD_HandleTypeDef usbdHandle;
extern USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops;
int stopped = 0;
int enable_print = 0;
static int tskStrtd = 0;
/* Definitions for CLI_manager */
osThreadId_t CLI_managerHandle;
/* Definitions for defaultTask */
osThreadId_t appStartupTaskHandle;
osThreadId_t wdtManagerTaskHandle;
osThreadId_t audioManagerTaskHandle;
osThreadId_t sensorManagerHandle;
osThreadId_t command_appHandle;
osThreadId_t systemManagerHandle;

IWDG_HandleTypeDef hiwdg;
uint32_t task_alive = 0;
uint32_t task_list = 0;
uint8_t avail_sensor_values = 0;
SemaphoreHandle_t wakeSensorTask, exitSleepMode, startCLI;
volatile bool resetPerformed = false;
QueueHandle_t eventQueue;
int startMonitoringValue = 0;
int goToSleep = 0;
//int sleepTimeout = 40000;
int sleepTimeout = 10000;
int noAcknowledgment = 0; /** A flag that indicates whether to send an acknowledgment or not (0: ACK, 1: NACK) */
SemaphoreHandle_t sleepSem;

extern uint16_t low_th;
extern uint16_t high_th;

const osThreadAttr_t CLI_manager_attributes = { .name = CLI_MANAGER_TASK_NAME,
		.stack_size = CLI_MANAGER_STK_SIZE, .priority =
				(osPriority_t) CLI_MANAGER_TASK_PRIO, };

const osThreadAttr_t startupTask_attributes = { .name = APP_STARTUP_TASK_NAME,
		.stack_size = APP_STARTUP_TASK_STK_SIZE, .priority =
				(osPriority_t) APP_STARTUP_TASK_PRIO, };

const osThreadAttr_t wdtManagerTask_attributes = {
		.name = WDT_MANAGER_TASK_NAME, .stack_size = WDT_MANAGER_TASK_STK_SIZE,
		.priority = (osPriority_t) WDT_MANAGER_TASK_PRIO, };

const osThreadAttr_t audioManagerTask_attributes = { .name =
AUDIO_MANAGER_TASK_NAME, .stack_size = AUDIO_MANAGER_TASK_STK_SIZE, .priority =
		(osPriority_t) AUDIO_MANAGER_TASK_PRIO, };

const osThreadAttr_t sensorManagerTask_attributes = { .name =
SENSOR_MANAGER_TASK_NAME, .stack_size = SENSOR_MANAGER_TASK_STK_SIZE,
		.priority = (osPriority_t) SENSOR_MANAGER_TASK_PRIO, };

const osThreadAttr_t command_app_attributes = { .name = COMMAND_APP_TASK_NAME,
		.stack_size = COMMAND_APP_TASK_STK_SIZE, .priority =
				(osPriority_t) COMMAND_APP_TASK_PRIO, };

osSemaphoreId_t wdt_semHandle;
const osSemaphoreAttr_t wdt_sem_attributes = { .name = "wdt_sem" };
/* USER CODE BEGIN PV */
static IOStates_t uartCaptureState = STATE_IDLE;
static int pktcount = 0;
unsigned char proc_buffer[PROC_BUFFER_SIZE];
int proc_data_flag = 0;
volatile uint8_t enable_console = FALSE;
int data_flag_bit = 0;

extern int prox_enabled;

int processReceivedPacket(unsigned char rx_byte);
RTC_HandleTypeDef hrtc;
extern USBD_DescriptorsTypeDef Class_Desc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART2_UART_Init(void);
void CLImanager(void *argument);
void AppStartup(void *argument);
void AudioManager(void *argument);
void SensorManager(void *argument);
void systemManager(void *argument);
void WdtManager(void *argument);
void StartTask_command_app(void *argument);
int app_mgr();
void Init_Acquisition_Peripherals(uint32_t AudioFreq, uint32_t ChnlNbrIn,
		uint32_t BitRes);
static void MX_USBD_Init(void);
static void MX_IWDG_Init(void);
int wdt_register(unsigned int task_id);
int wdt_keepalive(unsigned int wdt_task_id);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* To gives values to the fru struct and to call write function */
int flash_init_fru(char *board_type_t, char *serial_number_t,
		char *magic_number_t);
/* To gives values to the config struct and to call write function */
int flash_init_config(char *magic_num_t, char *config_ver_t,
		char *backlight_val_t, char *crc_t);
static void MX_RTC_Init(void);

void hub_init(void);

bool backButtonSet = false;
bool okButtonSet = false;
bool powerButtonSet = false;
bool powerSavingOn = false;
bool powerUpInterrupt = false;
bool firstTimeBoot = false;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

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
	MX_USART2_UART_Init();
	RetargetInit(&huart2);

	printf("VERSION: %d.%d.%d\r\n", SOFTWARE_VERSION_MAJOR,
	SOFTWARE_VERSION_MINOR, SOFTWARE_VERSION_PATCH);

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_I2C3_Init();
	MX_USBD_Init();
	MX_IWDG_Init();
	MX_RTC_Init();
	MX_SPI2_Init();
	MX_SPI1_Init();

	/* Initialize the hub via I2C so that USB2.0 is detected on the host */
	hub_init();
	/* Read the data stored in the flash memory */
	flash_read_FRU(&sys_info_read);

	firstTimeBoot = checkFirstTimeBoot_b();

	if (firstTimeBoot) {
#ifdef ENABLE_PRINTS
		printf("First time boot\n");
#endif
		firstTimeBoot = false;
		setDefaultConfigSettings_v();
		Flash_Write_FRU(&sys_info_write);
	}
	HAL_Delay(250);
	flash_read_FRU(&sys_info_read);

	HAL_Delay(500);
	/* Set the display configuration during boot (brightness, luminescence */
	Display_Configuration(0);

	HAL_Delay(1000);
	readSleepTimeout_v();

	/* Enable the button pins IRQn (power button, ok button & back button */
	/*
	 * EXTI3 - Power button
	 * EXTI4 - OK button
	 * EXTI6 - Back button
	 */
	enableButtonInterrupts_v();

	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();
	wdt_semHandle = osSemaphoreNew(1, 1, &wdt_sem_attributes);
	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* Initialize all configured peripherals */
	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	appStartupTaskHandle = osThreadNew(AppStartup, NULL,
			&startupTask_attributes);
	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

void readSleepTimeout_v(void) {
	sleepTimeout = (int) sys_info_read.config[SLEEP_TIM] * 1000;
}

void enableButtonInterrupts_v(void) {
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void setDefaultConfigSettings_v(void) {
	sys_info_write.config[BACKL] = 0;
	sys_info_write.config[1] = (uint32_t) 1;
	sys_info_write.config[2] = 0;
	sys_info_write.config[SLEEP_TIM] = (uint32_t) 20;
}

bool checkFirstTimeBoot_b(void) {
	bool firstTimeBootStatus = false;
	/* Check if the firstTimeBoot flag is set or not */
	if (sys_info_read.config[1] != 1) {
		/* Booting for the first time */
		firstTimeBootStatus = true;
	}
	return firstTimeBootStatus;
}

void hub_init(void) {
	uint8_t buf[10] = { 0 };

	//Setting GPIO 7 direction as output
	uint8_t mem_addr[2] = { 0 };
	uint8_t rec_buf[200] = { 0 };
	uint8_t value = 0;
	/* reading 411d register */

	if (HAL_I2C_IsDeviceReady(&hi2c3, HUB_DEV_ADDRESS, 10, 10)) {
		printf("HUB NOT DETECTED\r\n");
	}

	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x04;
	buf[3] = 0x01;
	buf[4] = 0x01;
	buf[5] = 0x41;
	buf[6] = 0x1D;

	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 7,
			100)) {
		printf("command failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
		printf("command execute failed\r\n");
	}

	memset(rec_buf, 0, sizeof(rec_buf));
	buf[0] = 0x00;
	buf[1] = 0x04;
	if (HAL_I2C_Mem_Read(&hi2c3, HUB_DEV_ADDRESS, 0x0004, 2, rec_buf, 3, 100)) {
		printf("command read failed\r\n");
	}

	/* setting bypass bit */
	value = rec_buf[1]; //getting value of the register 411Dh

//	int i = 0;
//	while(i < 5){
//		printf("ret: %d %x\r\n", i, rec_buf[i]);
//		i++;
//	}

	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x05;
	buf[3] = 0x00;
	buf[4] = 0x01;
	buf[5] = 0x41;
	buf[6] = 0x1D;
	buf[7] = (value | 0x08);

	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 8,
			100)) {
		printf("Register set failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
		printf("Register set execute failed\r\n");
	}

	/* checking for proper set */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x04;
	buf[3] = 0x01;
	buf[4] = 0x01;
	buf[5] = 0x41;
	buf[6] = 0x1D;

	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 7,
			100)) {
		printf("command failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
		printf("command execute failed\r\n");
	}

	memset(rec_buf, 0, sizeof(rec_buf));
	buf[0] = 0x00;
	buf[1] = 0x04;
	if (HAL_I2C_Mem_Read(&hi2c3, HUB_DEV_ADDRESS, 0x0004, 2, rec_buf, 3, 100)) {
		printf("command read failed\r\n");
	}

	//	int i=0;
//	i =0;
//	while(i < 5){
//		printf("ret: %d %x\r\n", i, rec_buf[i]);
//		i++;
//	}

	/* attach command with smb AA56 */
	buf[0] = 0xAA;
	buf[1] = 0x56;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
		printf("attach failed\r\n");
	}

	/* gpio 9 direction */
	buf[0] = 0x00;
	buf[1] = 0x00;
	buf[2] = 0x05;	//No. of bytes to write in memory
	buf[3] = 0x00;
	buf[4] = 0x01;	//No. of bytes to write
	buf[5] = 0x08;	//GPIO DIRECTION register
	buf[6] = 0x32;
	buf[7] = 0x02;

	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 8,
			100)) {
		printf("Direction set failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
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
	buf[7] = 0x00;	//setting as 0

	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 8,
			100)) {
		printf("GPIO set failed\r\n");
	}

	buf[0] = 0x99;
	buf[1] = 0x37;
	buf[2] = 0x00;
	if (HAL_I2C_Master_Transmit(&hi2c3, HUB_DEV_ADDRESS, (uint8_t*) buf, 3,
			100)) {
		printf("GPIO set command execute failed\r\n");
	}
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */
	//
	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */
	//
	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
//    hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&hi2c1);
	/* USER CODE BEGIN I2C1_Init 2 */
	//
	/* USER CODE END I2C1_Init 2 */

}
/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	gpio_output(SH_RSTN_GPIO_Port, SH_RSTN_Pin | SH_BOOTN_Pin, GPIO_NOPULL,
	GPIO_SPEED_FREQ_LOW);

	gpio_interrupt(SH_INTN_GPIO_Port, SH_INTN_Pin, GPIO_PULLUP,
	GPIO_MODE_IT_FALLING);

	// TODO: Check the below configurations once & verify with hardware
	/****** BACK, OK & POWER BUTTON INTERRUPT PINS & SETUP ******/
	gpio_interrupt(GPIOC, BACK_BUTTON_PIN, GPIO_PULLUP,
	GPIO_MODE_IT_FALLING);

	gpio_interrupt(GPIOC, OK_BUTTON_PIN, GPIO_PULLUP,
	GPIO_MODE_IT_FALLING);

	gpio_interrupt(GPIOC, POWER_BUTTON_PIN, GPIO_PULLUP,
	GPIO_MODE_IT_FALLING);
	/****** BACK, OK & POWER BUTTON INTERRUPT PINS & SETUP ******/

	// TODO: Set IRQn priorities for all the above interrupt handlers
	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI1_IRQn, IMU_INT_PRIORITY, 0);

	gpio_output(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN | DISP_1_8V_GPIO_PIN,
	GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	set_gpio_pin(DISP_XCLR_GPIO_PORT, DISP_XCLR_GPIO_PIN);
	reset_gpio_pin(DISP_XCLR_GPIO_PORT, DISP_1_8V_GPIO_PIN);

	gpio_output(LT_GPIO1_PORT, LT_GPIO1_PIN, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
//  	set_gpio_pin(LT_GPIO1_PORT, LT_GPIO1_PIN);
	reset_gpio_pin(LT_GPIO1_PORT, LT_GPIO1_PIN); /* default 2D */

	/*PB0 - LT reset */
	gpio_output(LT_RESET_PORT, LT_RESET_PIN, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	set_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);
//	reset_gpio_pin(LT_RESET_PORT, LT_RESET_PIN);  // for time being

	gpio_output(L_DISP_NSS_PORT, L_DISP_NSS_PIN, GPIO_PULLUP,
	GPIO_SPEED_FREQ_VERY_HIGH);
	set_gpio_pin(L_DISP_NSS_PORT, L_DISP_NSS_PIN);

//  	gpio_output(HUB_RESET_PORT, HUB_RESET_PIN, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
//  	set_gpio_pin(HUB_RESET_PORT, HUB_RESET_PIN);
////  	reset_gpio_pin(HUB_RESET_PORT, HUB_RESET_PIN);		//for the time being

	gpio_output(R_DISP_NSS_PORT, R_DISP_NSS_PIN, GPIO_PULLUP,
	GPIO_SPEED_FREQ_VERY_HIGH);
	set_gpio_pin(R_DISP_NSS_PORT, R_DISP_NSS_PIN);
}

/* USER CODE BEGIN 4 */

/**
 * @brief USBD Initialization Function
 * @param None
 * @retval None
 */
static void MX_USBD_Init(void) {
	USBD_AUDIO_Init_Microphone_Descriptor(&usbdHandle,
	AUDIO_IN_SAMPLING_FREQUENCY, AUDIO_IN_CHANNELS);
	USBD_Init(&usbdHandle, &Class_Desc, 0);
	USBD_RegisterClass(&usbdHandle, &USBD_AUDIO);
	USBD_AUDIO_RegisterInterface(&usbdHandle, &USBD_AUDIO_fops);
	USBD_Start(&usbdHandle);
}

/**
 * @brief IWDG Initialization Function
 * @param None
 * @retval None
 */
static void MX_IWDG_Init(void) {

	/* USER CODE BEGIN IWDG_Init 0 */

	/* USER CODE END IWDG_Init 0 */

	/* USER CODE BEGIN IWDG_Init 1 */

	/* USER CODE END IWDG_Init 1 */
	/* Set counter reload value to obtain 500ms IWDG TimeOut.
	 IWDG counter clock Frequency = LsiFreq / 4
	 Counter Reload Value = .5s / IWDG counter clock period
	 = .5 / (4/LsiFreq)
	 = LsiFreq / (4 * 2)
	 = LsiFreq / 8 */
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
	hiwdg.Init.Reload = 4000;

	/* USER CODE BEGIN IWDG_Init 2 */

	/* USER CODE END IWDG_Init 2 */

}

static void MX_I2C3_Init(void) {

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
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void) {
	// TODO: About the system up time, what has to be done? Will the AjnaPack keep track of it using USB communication?
	// Difficult on the hardware as the write cycles are limited

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	RTC_TimeTypeDef sTime = { 0 };
	RTC_DateTypeDef sDate = { 0 };

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */
	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 124;
	hrtc.Init.SynchPrediv = 7999;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK) {
		Error_Handler();
	}

	/* USER CODE BEGIN Check_RTC_BKUP */
	if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == RTC_MAGIC_NO) {
		return;
	}

	/* USER CODE END Check_RTC_BKUP */

	/** Initialize RTC and set the Time and Date
	 */
	sTime.Hours = 0x0;
	sTime.Minutes = 0x0;
	sTime.Seconds = 0x0;
	sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	sDate.WeekDay = RTC_WEEKDAY_FRIDAY;
	sDate.Month = RTC_MONTH_JANUARY;
	sDate.Date = 0x1;
	sDate.Year = 0x21;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_MAGIC_NO); // backup register

	/* USER CODE END RTC_Init 2 */
}

/**
 * @brief Initialize all the necessary tasks using th
 */
void AppStartup(void *argument) {
	/* Keeps checking if the watchdog is refreshed by all the tasks on time or not */
	wdtManagerTaskHandle = osThreadNew(WdtManager, NULL,
			&wdtManagerTask_attributes);

	IMU_Init(&hi2c1);

	/* Used to handle interrupts from the IMU & deals with proximity sensor reads */
	sensorManagerHandle = osThreadNew(SensorManager, NULL,
			&sensorManagerTask_attributes);

	/** Used to receive commands over USB2.0 endpoints (command endpoint & custom endpoint */
	command_appHandle = osThreadNew(StartTask_command_app, NULL,
			&command_app_attributes);

	vTaskDelete(NULL);
}
/**
 * @brief  Function implementing the task2 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask2 */
void WdtManager(void *argument) {
	wdt_register(WDT_MANAGER_TASK_ID);
	int wdt_counter = 0;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
	}
	HAL_IWDG_Refresh(&hiwdg);
	osSemaphoreRelease(wdt_semHandle);
#ifdef ENABLE_PRINTS
	printf("wdt started\r\n");
#endif
	while (1) {
		wdt_keepalive(WDT_MANAGER_TASK_ID);
		osDelay(100 * WDT_FEED_INTERVAL);
		if (wdt_counter < 50) {
			wdt_counter++;
			HAL_IWDG_Refresh(&hiwdg);
		}
		if (wdt_counter >= 50) {
			//if all tasks have sent alive signal
			if (task_list == task_alive) {
				osSemaphoreAcquire(wdt_semHandle, 0);
				wdt_counter = 0;
				task_alive = 0;
				HAL_IWDG_Refresh(&hiwdg);
				osSemaphoreRelease(wdt_semHandle);
			} else {
				printf("will reboot wdt tsk list %d alv %d\r\n", task_list,
						task_alive);
				osDelay(500);
			}
		}
	}
	/* USER CODE END 5 */
}

/**
 * @brief  Function implementing the task2 thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask2 */
void SensorManager(void *argument) {
    wdt_register(SENSOR_TASK_ID);

	sh2_SensorEvent_t sensorEvent;
	uint16_t retval = 0;

	static int prox_count = 0;
	int enable = 0;
	retval = start_Prox_Sensor();
	if (retval != OK) {
		printf("unable to start ProxSensor");
	}
	wakeSensorTask = xSemaphoreCreateBinary();

	eventQueue = xQueueCreate(SENSOR_EVENT_QUEUE_SIZE,
			sizeof(sh2_SensorEvent_t));
	if (eventQueue == NULL) {
		printf("Error creating event queue.\n");
	}
	resetPerformed = false;
	//startedReports = false;

	// init SH2 layer
	sh2_initialize(eventHandler, NULL);

	// Register event listener
	sh2_setSensorCallback(sensorHandler, NULL);

	// wait for reset notification, or just go ahead after 100ms
	int waited = 0;
	while (!resetPerformed && (waited < 500)) {
		vTaskDelay(1);
		waited++;
	}

	onReset();

	xSemaphoreTake(wakeSensorTask, portMAX_DELAY);

#ifdef ENABLE_PRINTS
	printf("sensor manager started\r\n");
#endif
	while (1) {
		wdt_keepalive(SENSOR_TASK_ID);
		if ((prox_count % 100) == 0) {
			/*Since proximity is used only for sleep Mod-100 to reduce the times of read*/
			IMU_SensorData.prox_data = prox_read();
		}
		if (xQueueReceive(eventQueue, &sensorEvent, (TickType_t)10) == pdTRUE) {
			receiveEvent(&sensorEvent);
		}
#ifdef ENABLE_SLEEP_MODE
		processProximityData_v();
#endif

		osDelay(1);
		prox_count++;
	}
}

/**
 * @brief A function that reads proximity data periodically and sends the device to sleep if the value is below the threshold
 * TODO: Could be prone to bugs, not tested on hardware
*/
void processProximityData_v() {
	static uint32_t monitorTckCount = 0;
	if (IMU_SensorData.prox_data <= 8) {
		startMonitoringValue = 1;
		if (startMonitoringValue) {
			if (xTaskGetTickCount() - monitorTckCount >= sleepTimeout) {
				startMonitoringValue = 0;
				enterSleep();
			}
		}
	} else {
		/* Do nothing, function normally */
		startMonitoringValue = 0;
		monitorTckCount = xTaskGetTickCount();
	}
}

/**
 * @brief Function implementing the command_app thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask_command_app */
void StartTask_command_app(void *argument) {
#ifdef ENABLE_PRINTS
	printf("command app strtd\r\n");
#endif
	wdt_register(COMMAND_APP_TASK_ID);

	/* USER CODE BEGIN StartTask_command_app */
	uint8_t buffer[128] = { 0 };
	const size_t xStreamBufferSizeBytes = 128, xTriggerLevel = 1;

	/*Create a stream buffer that can hold 100 bytes. */
	xStreamBuffer = xStreamBufferCreate(xStreamBufferSizeBytes, xTriggerLevel);
	if (xStreamBuffer == NULL) {
		/* There was not enough heap memory space available to create the stream buffer.
		 * Task loops here */
		printf("Unable to create Command App buffer");
		return;
	}
	/* Infinite loop */
	for (;;) {
		wdt_keepalive(COMMAND_APP_TASK_ID);
		/* Checks if any buttons are pressed or not (flags are set within the respective EXTI handlers */
		if (backButtonSet) {
			backButtonSet = false;
			IMU_SensorData.backButtonFlag = 1;
		} else if (okButtonSet) {
			okButtonSet = false;
			IMU_SensorData.okButtonFlag = 1;
		} else if (powerButtonSet) {
			powerButtonSet = false;
			IMU_SensorData.powerButtonFlag = 1;
		}

		/* The below is used when the device (glasses) are in sleep mode & the power
		 * buton is pressed on the glasses to wake the device up
		 */
		if (powerUpInterrupt) {
			powerUpInterrupt = false;
			exitSleep();
		}
		/* Receives data from USB 2.0 */
		size_t count = xStreamBufferReceive(xStreamBuffer, buffer, 128, 2);
		if (count == 0) {
			osDelay(100);
			continue;
		}

		if (parse_buffer(buffer, &count)) {
			while (1)
				; //NULL passed, task loops
		}
		osDelay(100);
		for (;;) {
			/* noAcknowledgment is set only during sleep commands execution to avoid
			 * unnecessary acknowledgments */
			if (noAcknowledgment != 1) {
				uint8_t status = USBD_Command_Transmit(&usbdHandle, buffer,
						count);
				if (status == USBD_OK) {
					break;
				}
			} else {
				break;
			}
			osDelay(2);
		}
	}
	/* USER CODE END StartTask_command_app */
}
/* USER CODE END 4 */

/**
 * @brief This function handles I2C1 event interrupt.
 */
void I2C1_EV_IRQHandler(void) {
	HAL_I2C_EV_IRQHandler(&hi2c1);
}

/**
 * @brief This function handles I2C1 error interrupt.
 */
void I2C1_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hi2c1);
}

/**
 * @brief If any sensor reports are ready from BNO080, the below function will be called &
 * based on the event type, the switch case will be handled
 */
void receiveEvent(const sh2_SensorEvent_t *event) {
	static uint8_t idx = 0;
	int rc;
	uint16_t retval = 0;
	sh2_SensorValue_t value;
	float scaleRadToDeg = 180.0 / 3.14159265358;
	float r, i, j, k, acc_deg, x, y, z;
	float t;

	rc = sh2_decodeSensorEvent(&value, event);
	if (rc != SH2_OK) {
		printf("parsing err\r\n");
		return;
	}

	// t = value.timestamp / 1000.0;  // time in seconds.

	switch (value.sensorId) {

	case SH2_ROTATION_VECTOR:
		IMU_SensorData.RV_real[idx] = value.un.rotationVector.real;
		IMU_SensorData.RV_i[idx] = value.un.rotationVector.i;
		IMU_SensorData.RV_j[idx] = value.un.rotationVector.j;
		IMU_SensorData.RV_k[idx] = value.un.rotationVector.k;

		idx++;
		if (idx == PACK_SIZE) 
		{
			IMU_SensorData.timestamp = rtc_read();
			if (USBD_CUSTOM_HID_SendReport(&usbdHandle, (uint8_t*) &IMU_SensorData, sizeof(IMU_SensorData)) != USBD_OK) {}
			idx = 0;
		}

		break;

	case SH2_ARVR_STABILIZED_RV:
		IMU_SensorData.AR_RV_real[idx] = value.un.arvrStabilizedRV.real;
		IMU_SensorData.AR_RV_i[idx] = value.un.arvrStabilizedRV.i;
		IMU_SensorData.AR_RV_j[idx] = value.un.arvrStabilizedRV.j;
		IMU_SensorData.AR_RV_k[idx] = value.un.arvrStabilizedRV.k;


		break;

	case SH2_ACCELEROMETER:
		IMU_SensorData.acc_x = value.un.accelerometer.x;
		IMU_SensorData.acc_y = value.un.accelerometer.y;
		IMU_SensorData.acc_z = value.un.accelerometer.z;
		break;

	case SH2_MAGNETIC_FIELD_CALIBRATED:
		IMU_SensorData.mag_x = value.un.magneticField.x;
		IMU_SensorData.mag_y = value.un.magneticField.y;
		IMU_SensorData.mag_z = value.un.magneticField.z;
		break;

	case SH2_GYROSCOPE_CALIBRATED:
		IMU_SensorData.gyro_x = value.un.gyroscope.x;
		IMU_SensorData.gyro_y = value.un.gyroscope.y;
		IMU_SensorData.gyro_z = value.un.gyroscope.z;
		break;

	case SH2_GAME_ROTATION_VECTOR:
		IMU_SensorData.AR_RV_real[idx] = value.un.gameRotationVector.real;
		IMU_SensorData.AR_RV_i[idx] = value.un.gameRotationVector.i;
		IMU_SensorData.AR_RV_j[idx] = value.un.gameRotationVector.j;
		IMU_SensorData.AR_RV_k[idx] = value.un.gameRotationVector.k;

		idx++;
		if (idx == PACK_SIZE) {
			IMU_SensorData.timestamp = rtc_read();
			if (USBD_CUSTOM_HID_SendReport(&usbdHandle,
					(uint8_t*) &IMU_SensorData, sizeof(IMU_SensorData))
					!= USBD_OK) {
			}
			idx = 0;
		}
		break;

		/*
		 * The stability sensor is activated when the device is put into sleep mode &
		 * is woken up when there is a significant motion on the device
		 */
	case SH2_SIGNIFICANT_MOTION: {
#ifdef ENABLE_PRINTS
		printf("%8.4f motion detected %d\r\n", t, value.un.sigMotion.motion);
#endif
		/* Turns on the various IMU sensors & turns off the stability sensor */
		exitSleep();
		break;
	}

	default:
		printf("Unknown sensor: %d\n", value.sensorId);
		break;
	}
}

void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent) {
	if (xQueueSend(eventQueue, pEvent, 0) != pdPASS) {}
	xSemaphoreGive(wakeSensorTask);
}

static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_LSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
	hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM1) {
		if (enable_iwdg_refresh) {
			HAL_IWDG_Refresh(&hiwdg);
		}
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/******************************************************************************
 * @fn	   : wdt_register
 * @brief  : This function registers whenever the new task gets initialized
 * @return : SUCCESS        - When all the task get registered
 *		     ERR_UPDATE_WDT - returns error signal when registration
 *							  of task gets timeout
 ******************************************************************************/
int wdt_register(unsigned int task_id) {
	osSemaphoreAcquire(wdt_semHandle, 0);
	task_list = task_list | task_id;
	osSemaphoreRelease(wdt_semHandle);
	return 0;
}

/******************************************************************************
 * @fn	   : wdt_keepalive
 * @brief  : This function updates the task presence for certain time interval
 * @return : SUCCESS        - When all the task updates regularly
 *		 	 ERR_UPDATE_WDT - returns error signal when any
 *							of the registered task is not alive
 ******************************************************************************/
int wdt_keepalive(unsigned int wdt_task_id) {
	osSemaphoreAcquire(wdt_semHandle, 0);
	task_alive = task_alive | wdt_task_id;
	osSemaphoreRelease(wdt_semHandle);
	return 0;
}

void wakeup() {
//	 HAL_ResumeTick();
	enable_iwdg_refresh = 0;
	powerSavingOn = false;
//	HAL_PWR_DisableSleepOnExit();
	stopped = 0;
	// xSemaphoreGive(exitSleepMode);
}

void enterSleep() {
	int enable = 0;

	powerSavingOn = 1;
	clearDisplayLT7211B_v();
	resetLT7211B_v(true);

	enable_acc_t(&enable, &enable, &enable);
	enable_gyro_t(&enable, &enable, &enable);
	enable_mag_t(&enable, &enable, &enable);
	enable_RV_t(&enable, &enable, &enable);
	goToSleep = 0;
	// TODO: Uncomment this when proximity sensor is available
//	if (set_TH(low_th, high_th) != OK) {
//		printf("Prox-Threshold: Set Failed");
//	}
//	enable_Prox_int();

	enableStabilitySensor();
	xQueueReset(eventQueue);
	printf("entered the sleep mode \r\n");
	stopped = 1;
	enable_iwdg_refresh = 1;
}

void exitSleep() {
#ifdef ENABLE_PRINTS
	printf("wakeup \r\n");
#endif
	int enable = 1;

	/** Sends an event into the evtQueue */
	imu_INT_EVT();
	resetLT7211B_v(false);
	osDelay(100);
	clearDisplayLT7211B_v();
	osDelay(100);
	Display_Configuration(0);
	osDelay(100);
	wakeup();

	powerSavingOn = false;
	disable_prox_int();

	disableStabilitySensor();

	enable_acc_t(&enable, &enable, &enable);
	enable_gyro_t(&enable, &enable, &enable);
	enable_mag_t(&enable, &enable, &enable);
	enable_RV_t(&enable, &enable, &enable);
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
