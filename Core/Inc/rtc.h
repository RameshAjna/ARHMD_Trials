/*
 * rtc.h
 *
 *  Created on: 29-Sep-2021
 *      Author: vvdn
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "stm32f4xx_hal.h"
#include "time.h"

#define JULIAN_DATE_BASE	2440588   // Unix epoch time in Julian calendar (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)
#define RTC_MAGIC_NO		0x32F2

/**APIs**/

// Get Date and Time in Structures
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

// Get epoch time
time_t rtc_read(void);

// Convert Date/Time structures to epoch time
//uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
uint32_t RTC_ToEpoch();

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

#endif /* INC_RTC_H_ */
