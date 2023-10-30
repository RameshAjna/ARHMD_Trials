/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file sh2_hal_i2c.c
 * @author David Wheeler
 * @date 18 Nov 2016
 * @brief SH2 HAL Implementation for BNO080, via I2C on STM32F411re Nucleo board
 *        with FreeRTOS.
 */

#include "main.h"
#include "sh2_hal.h"
#include "shtp.h"
#include "sh2_err.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "queue.h"
//#include "semphr.h"
#include "sh2.h"
#include "sh2_SensorValue.h"

#define DFU_BOOT_DELAY (200) // [mS]
#define RESET_DELAY    (10) // [mS]

#define MAX_EVENTS (25)
#define SHTP_HEADER_LEN (4)

#define ADDR_DFU_0 (0x28)
#define ADDR_DFU_1 (0x29)
#define ADDR_SH2_0 (0x4A)
#define ADDR_SH2_1 (0x4B)

#define RSTN_GPIO_PORT GPIOC
#define RSTN_GPIO_PIN  GPIO_PIN_0

#define BOOTN_GPIO_PORT GPIOC
#define BOOTN_GPIO_PIN  GPIO_PIN_1

// ----------------------------------------------------------------------------------
// Forward declarations
// ----------------------------------------------------------------------------------
void IMU_Task(void *params);
static void i2cReset(void);
static int i2cBlockingRx(unsigned addr, uint8_t *pData, unsigned len);
static int i2cBlockingTx(unsigned addr, uint8_t *pData, unsigned len);
static void rstn0(bool state);
static void bootn0(bool state);
extern int goToSleep;

#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#endif

// Define this to perform fimware update at startup.
// #define PERFORM_DFU

// Define this to use HMD-appropriate configuration.
// #define CONFIGURE_HMD

// Define this for calibration config appropriate for Robot Vaccuum Cleaners
// #define CONFIGURE_RVC

#ifdef PERFORM_DFU
#include "dfu_bno080.h"
#include "firmware.h"
#endif

#define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))
const float scaleDegToRad = 3.14159265358 / 180.0;

// --- Forward declarations -------------------------------------------

// --- Private data ---------------------------------------------------

sh2_ProductIds_t prodIds;



//SemaphoreHandle_t wakeSensorTask;

extern volatile bool resetPerformed;
extern SemaphoreHandle_t wakeSensorTask;
volatile bool startedReports = false;

QueueHandle_t eventQueue;

// --- Public methods -------------------------------------------------

/* * */
int condition = 0;
int condition_reset = 1;
extern int stopped;

// ----------------------------------------------------------------------------------
// Private data
// ----------------------------------------------------------------------------------

// I2C Bus access
static I2C_HandleTypeDef *hi2c;
static SemaphoreHandle_t i2cMutex;
static SemaphoreHandle_t i2cBlockSem;
int i2cStatus;
bool i2cResetNeeded;

// HAL Queue and Task
static QueueHandle_t evtQueue = 0;
osThreadId halTaskHandle;

typedef struct {
    void (*rstn)(bool);
    void (*bootn)(bool);
    sh2_rxCallback_t *onRx;
    void *onRxCookie;
    uint16_t addr;
    uint8_t rxBuf[SH2_HAL_MAX_TRANSFER];
    uint16_t rxRemaining;
    SemaphoreHandle_t blockSem;
} Sh2Hal_t;
Sh2Hal_t sh2Hal;

typedef enum {
    EVT_INTN,
} EventId_t;

typedef struct {
    uint32_t t_ms;
    EventId_t id;
} Event_t;

// ----------------------------------------------------------------------------------
// Public API
// ----------------------------------------------------------------------------------
// Initialize SH-2 HAL subsystem
void IMU_Init(I2C_HandleTypeDef *i2c) {

    // Store reference to I2C peripheral
    hi2c = i2c;

    // Need to init I2C peripheral before first use.
    i2cResetNeeded = true;

    // Initialize SH2 HAL data structure
    memset(&sh2Hal, 0, sizeof(sh2Hal));

    // Semaphore to block clients with block/unblock API
    sh2Hal.blockSem = xSemaphoreCreateBinary();

    sh2Hal.rstn = rstn0;
    sh2Hal.bootn = bootn0;

    // Put SH2 device in reset
    sh2Hal.rstn(false);  // Hold in reset
    sh2Hal.bootn(true);  // SH-2, not DFU

    // init mutex for i2c bus
    i2cMutex = xSemaphoreCreateMutex();
    i2cBlockSem = xSemaphoreCreateBinary();

    // Create queue to pass events from ISRs to task context.
    evtQueue = xQueueCreate(MAX_EVENTS, sizeof(Event_t));
    if (evtQueue == NULL) {
        printf("The queue could not be created.\n");
    }

    /* Definitions for defaultTask */
    const osThreadAttr_t halThread_attributes ={
        .name = IMU_APP_TASK_NAME,
        .stack_size = IMU_APP_STK_SIZE,
        .priority = (osPriority_t) IMU_TASK_TASK_PRIO, };

    halTaskHandle = osThreadNew(IMU_Task, NULL, &halThread_attributes);

    if (halTaskHandle == NULL) {
        printf("Failed to create SH-2 HAL task.\n");
    }
}

// Reset an SH-2 module (into DFU mode, if flag is true)
// The onRx callback function is registered with the HAL at the same time.
int sh2_hal_reset(bool dfuMode, sh2_rxCallback_t *onRx, void *cookie) {
    // Get exclusive access to i2c bus (blocking until we do.)
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Store params for later reference
    sh2Hal.onRxCookie = cookie;
    sh2Hal.onRx = onRx;

    // Set addr to use in this mode
    sh2Hal.addr = dfuMode ? ADDR_DFU_0 << 1 : ADDR_SH2_0 << 1;

    // Assert reset
    sh2Hal.rstn(0);

    // Set BOOTN according to dfuMode
    sh2Hal.bootn(dfuMode ? 0 : 1);

    // Wait for reset to take effect
    vTaskDelay(RESET_DELAY);

    // Enable INTN interrupt
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    // Deassert reset
    sh2Hal.rstn(1);

    // If reset into DFU mode, wait until bootloader should be ready
    if (dfuMode) {
        vTaskDelay(DFU_BOOT_DELAY);
    }

    // Will need to reset the i2c peripheral after this.
    i2cResetNeeded = true;

    // Give up ownership of i2c bus.
    xSemaphoreGive(i2cMutex);
    return SH2_OK;
}

// Send data to SH-2
int sh2_hal_tx(uint8_t *pData, uint32_t len) {
    // Do nothing if len is zero
    if (len == 0) {
        return SH2_OK;
    }

    // Do tx, and return when done
    return i2cBlockingTx(sh2Hal.addr, pData, len);
}

// Initiate a read of <len> bytes from SH-2
// This is a blocking read, pData will contain read data on return
// if return value was SH2_OK.
int sh2_hal_rx(uint8_t *pData, uint32_t len) {
    // Do nothing if len is zero
    if (len == 0) {
        return SH2_OK;
    }

    // do rx and return when done
    return i2cBlockingRx(sh2Hal.addr, pData, len);
}

int sh2_hal_block(void) {
    xSemaphoreTake(sh2Hal.blockSem, portMAX_DELAY);

    return SH2_OK;
}

int sh2_hal_unblock(void) {
    xSemaphoreGive(sh2Hal.blockSem);

    return SH2_OK;
}

void disableStabilitySensor()
{
    sh2_SensorConfig_t config;
	int status;
	memset(&config, 0, sizeof(sh2_SensorConfig_t));
	status = sh2_setSensorConfig(SH2_SIGNIFICANT_MOTION, &config);
	if (status != 0) {
		printf("Error %d while disabling sensor %d\r\r\n", status, SH2_STABILITY_DETECTOR);
	}
	else
	{
		printf("disabled stability sensor\r\n");
	}
}

void enableStabilitySensor()
{
	static sh2_SensorConfig_t config;
	memset(&config, 0, sizeof(sh2_SensorConfig_t));
	int status;
	uint32_t frs_config[2] = {0};
	uint16_t size = (sizeof(frs_config)/sizeof(frs_config[0]));

	memset(frs_config, 0, sizeof(frs_config));
	status = sh2_getFrs((uint16_t)SIG_MOTION_DETECT_CONFIG, frs_config, &size);
	printf("got frs %ld, %ld\r\n", frs_config[0], frs_config[1]);
	if(status != SH2_OK)
	{
		printf("get frs failure\r\n");
	}
	if(frs_config[0] != FIX_Q(24, 1.0) || frs_config[1] != (uint32_t)1)
	{
		printf("set frs %ld, %ld\r\n", frs_config[0], frs_config[1]);
		frs_config[0] = FIX_Q(24, 1.0);
		frs_config[1] = (uint32_t)1;

		size = (uint16_t)2;
		status = sh2_setFrs(SIG_MOTION_DETECT_CONFIG, frs_config, size);
		if(status != SH2_OK)
		{
			printf("set frs failure\r\n");
		}
	}

	config.changeSensitivityEnabled = false;
	config.wakeupEnabled = true;
	config.changeSensitivityRelative = false;
	config.alwaysOnEnabled = false;
	config.changeSensitivity = 0;
	config.reportInterval_us = 10000;  // microseconds (100Hz)
	config.batchInterval_us = 0;
	sh2_setSensorConfig(SH2_SIGNIFICANT_MOTION, &config);
	if (status != 0) {
		printf("Error %d while enabling sensor %d\r\r\n", status, SH2_STABILITY_DETECTOR);
	}
	else
	{
		printf("enabled stability sensor\r\n");
	}
}


void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t woken = pdFALSE;
    // Set status from this operation
    i2cStatus = SH2_OK;
    // Unblock the caller
    xSemaphoreGiveFromISR(i2cBlockSem, &woken);
    portEND_SWITCHING_ISR(woken);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    BaseType_t woken = pdFALSE;
    // Set status from this operation
    i2cStatus = SH2_OK;

    // Unblock the caller
    xSemaphoreGiveFromISR(i2cBlockSem, &woken);
    portEND_SWITCHING_ISR(woken);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    BaseType_t woken = pdFALSE;
    // Set status from this operation
    i2cStatus = SH2_ERR_IO;

    // Unblock the caller
    xSemaphoreGiveFromISR(i2cBlockSem, &woken);
    portEND_SWITCHING_ISR(woken);
}

void imu_INT_EVT()
{
//	int start = HAL_GetTick();

	BaseType_t woken = pdFALSE;
	Event_t event;
	event.t_ms = xTaskGetTickCount();
	event.id = EVT_INTN;
	xQueueSendFromISR(evtQueue, &event, &woken);
	portEND_SWITCHING_ISR(woken);

//	int end = HAL_GetTick();
//	printf("%d\r\n", (end - start));
}
// ----------------------------------------------------------------------------------
// Private functions
// ----------------------------------------------------------------------------------

void IMU_Task(void *params)
{
    wdt_register(IMU_APP_TASK_ID);

	int start = 0, end = 0;

    Event_t event;
    unsigned readLen = 0;
    unsigned cargoLen = 0;

    while (1) {
    	wdt_keepalive(IMU_APP_TASK_ID);

    	if(xQueueReceive(evtQueue, &event, 10) == pdPASS)
        {
        // Handle the event
			switch (event.id) {
			case EVT_INTN:
//            	start = HAL_GetTick();

				// If no RX callback registered, don't bother trying to read
				if (sh2Hal.onRx != 0) {
					// Compute read length
					readLen = sh2Hal.rxRemaining;
					if (readLen < SHTP_HEADER_LEN) {
						// always read at least the SHTP header
						readLen = SHTP_HEADER_LEN;
					}
					if (readLen > SH2_HAL_MAX_TRANSFER) {
						// limit reads to transfer size
						readLen = SH2_HAL_MAX_TRANSFER;
					}

					// Read i2c
					i2cBlockingRx(sh2Hal.addr, sh2Hal.rxBuf, readLen);
//                    HAL_I2C_Master_Receive(hi2c, sh2Hal.addr, sh2Hal.rxBuf, readLen, 1000);

					// Get total cargo length from SHTP header
					cargoLen = ((sh2Hal.rxBuf[1] << 8) + (sh2Hal.rxBuf[0]))
							& (~0x8000);

//                    printf("%d\r\n", cargoLen);
					// Re-Evaluate rxRemaining
					if (cargoLen > readLen) {
						// More to read.
						sh2Hal.rxRemaining = (cargoLen - readLen)
								+ SHTP_HEADER_LEN;
//                        printf("%d\r\n", sh2Hal.rxRemaining);
					} else {
						// All done, next read should be header only.
						sh2Hal.rxRemaining = 0;
					}

					// Deliver via onRx callback
					sh2Hal.onRx(sh2Hal.onRxCookie, sh2Hal.rxBuf, readLen,
							event.t_ms * 1000);

					if ((sh2Hal.rxRemaining != 0)
							&& (sh2Hal.rxRemaining > SHTP_HEADER_LEN)) {
						readLen = sh2Hal.rxRemaining;

						i2cBlockingRx(sh2Hal.addr, sh2Hal.rxBuf, readLen);
//				  HAL_I2C_Master_Receive(hi2c, sh2Hal.addr, sh2Hal.rxBuf, readLen, 1000);

						// Get total cargo length from SHTP header
						cargoLen = ((sh2Hal.rxBuf[1] << 8) + (sh2Hal.rxBuf[0]))
								& (~0x8000);

//                    printf("%d\r\n", cargoLen);
						// Re-Evaluate rxRemaining
						if (cargoLen > readLen) {
							// More to read.
							sh2Hal.rxRemaining = (cargoLen - readLen)
									+ SHTP_HEADER_LEN;
//                        printf("%d\r\n", sh2Hal.rxRemaining);
						} else {
							// All done, next read should be header only.
							sh2Hal.rxRemaining = 0;
						}

						// Deliver via onRx callback
						sh2Hal.onRx(sh2Hal.onRxCookie, sh2Hal.rxBuf, readLen,
								event.t_ms * 1000);
					}
				}

//                end = HAL_GetTick();
//                printf("%d\r\n", end - start);
				break;
			default:
				// Unknown event type.  Ignore.
				break;
			} // End of Switch
        }
//        wdt_keepalive(IMU_APP_TASK_ID);
        osDelay(1);
    } // End of while
}

// Perform a blocking i2c read
static int i2cBlockingRx(unsigned addr, uint8_t *pData, unsigned len)
{
    int status = SH2_OK;

    // Get device mutex
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Reset bus, if necc.
    if (i2cResetNeeded) {
        i2cReset();
    }

    // Call I2C API rx
//    int rc = HAL_I2C_Master_Receive_IT(hi2c, addr, pData, len);
    int rc = HAL_I2C_Master_Receive(hi2c, addr, pData, len, 1000);

    if (rc == 0) {
        // Block on results
//        xSemaphoreTake(i2cBlockSem, portMAX_DELAY);

        // Set return status
        status = i2cStatus;
    } else {
        // I2C operation failed
        status = SH2_ERR_IO;
    }

    // Release device mutex
    xSemaphoreGive(i2cMutex);

    return status;
}

static int i2cBlockingTx(unsigned addr, uint8_t *pData, unsigned len) {
    int status = SH2_OK;
    // Get device mutex
    xSemaphoreTake(i2cMutex, portMAX_DELAY);

    // Reset bus, if necc.
    if (i2cResetNeeded) {
        i2cReset();
    }

    // Call I2C API rx
//    int rc = HAL_I2C_Master_Transmit_IT(hi2c, addr, pData, len);
    int rc = HAL_I2C_Master_Transmit(hi2c, addr, pData, len, 1000);

    if (rc == 0) {
        // Block on results
//        xSemaphoreTake(i2cBlockSem, portMAX_DELAY);

        // Set return status
        status = i2cStatus;
    } else {
        // I2C operation failed
        status = SH2_ERR_IO;
    }

    // Release device mutex
    xSemaphoreGive(i2cMutex);

    return status;
}

// DeInit and Init I2C Peripheral.
// (This recovery step is necessary after resets of the device)
static void i2cReset(void) {
    HAL_I2C_DeInit(hi2c);
    hi2c->Instance = I2C1;
//    hi2c->Init.ClockSpeed = 400000;
    hi2c->Init.ClockSpeed = 100000;
    hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    hi2c->Init.OwnAddress2 = 0;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

    HAL_I2C_Init(hi2c);

    i2cResetNeeded = false;
}

static void bootn0(bool state)
{
    HAL_GPIO_WritePin(BOOTN_GPIO_PORT, BOOTN_GPIO_PIN,
            state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void rstn0(bool state)
{
    HAL_GPIO_WritePin(RSTN_GPIO_PORT, RSTN_GPIO_PIN,
            state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 *	Hillcrest BNO080
 */

// --- Private methods ----------------------------------------------

void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    if (pEvent->eventId == SH2_RESET) {
        // Signal main loop to handle this.
        resetPerformed = true;
        // Signal main loop to handle this.
                resetPerformed = true;
                xSemaphoreGive(wakeSensorTask);
            }
}



#ifdef CONFIGURE_HMD
// Enable GIRV prediction for 28ms with 100Hz sync
#define GIRV_PRED_AMT FIX_Q(10, 0.028)             // prediction amt: 28ms
#else
// Disable GIRV prediction
#define GIRV_PRED_AMT FIX_Q(10, 0.0)               // prediction amt: 0
#endif

void flushReports()
{
	static const int enabledSensors[] =
	    {
//	        SH2_RAW_MAGNETOMETER,
//	        SH2_RAW_ACCELEROMETER,
//	        SH2_RAW_GYROSCOPE,
			SH2_ROTATION_VECTOR,
	        SH2_ARVR_STABILIZED_RV,
//			SH2_GYRO_INTEGRATED_RV,
	//        SH2_ACCELEROMETER,
	//        SH2_GYROSCOPE_CALIBRATED,
	 //       SH2_MAGNETIC_FIELD_CALIBRATED,
	    };
	for (int n = 0; n < ARRAY_LEN(enabledSensors); n++) {
	sh2_flush(enabledSensors[n]);
	}
}

 void startReports(void)
{
    static sh2_SensorConfig_t config;
    int status;
    int sensorId;
    static const int enabledSensors[] =
    {
//        SH2_ARVR_STABILIZED_RV,
        SH2_ACCELEROMETER,
        SH2_GYROSCOPE_CALIBRATED,
        SH2_MAGNETIC_FIELD_CALIBRATED,
		SH2_ROTATION_VECTOR,
		SH2_GAME_ROTATION_VECTOR
    };
    // dynamic calibration for A, G and M sensors
    status = sh2_setCalConfig(SH2_CAL_ACCEL);

	if (status != SH2_OK) {
		printf("Error: %d, from sh2_setCalConfig() in configure().\r\n", status);
	}

    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;

    config.batchInterval_us = 0;

    for (int n = 0; n < ARRAY_LEN(enabledSensors); n++) {
        // Configure the sensor hub to produce these reports
        sensorId = enabledSensors[n];
        if(SH2_ARVR_STABILIZED_RV == enabledSensors[n] || SH2_ROTATION_VECTOR == enabledSensors[n] || SH2_GAME_ROTATION_VECTOR == enabledSensors[n] || 
            SH2_GYRO_INTEGRATED_RV == enabledSensors[n])
        {
        	config.reportInterval_us = 2500;  //400 Hz
        }
        else
        {
        	config.reportInterval_us = 10000;  // 100 Hz
        }

        status = sh2_setSensorConfig(sensorId, &config);
        if (status != 0) {
            printf("Error %d while enabling sensor %d\r\r\n", status, sensorId);
        }
    }
}

void onReset(void)
{
    // Start the flow of sensor reports
    startReports();

    // Toggle reset flag back to false
    resetPerformed = false;
}
void clearEvtQueue()
{
	xQueueReset(evtQueue);
}
void reportProdIds(void)
{
    int status;

    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);

    if (status < 0) {
    	printf("getting prd ids failed\r\n");
        return;
    }

    // Report the results
    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %ld : Version %d.%d.%d Build %ld\r\n",
                prodIds.entry[n].swPartNumber,
                prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor,
                prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);
    }
}
