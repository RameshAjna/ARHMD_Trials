/*
 * rtc.c
 *
 *  Created on: 28-Sep-2021
 *      Author: vvdn
 */

#include "rtc.h"

extern RTC_HandleTypeDef hrtc;

static RTC_DateTypeDef dateStruct;
static RTC_TimeTypeDef timeStruct;
static struct tm timeinfo;

/*
 *Func: RTC_GetDateTime - Get current date and time
 *Input:
 *Time - pointer to RTC time structure
 *Date - pointer to RTC date structure
 *Retval: date and time in Time and Date structures
 * */
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	HAL_RTC_GetTime(&hrtc, time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, date, RTC_FORMAT_BIN);
}

/*
 *Func: rtc_read - Get current date and time in epoch
 *Input: None
 *Retval: returns epoch time
 * */
time_t rtc_read(void)
{
    hrtc.Instance = RTC;

    // Read actual date and time
    HAL_RTC_GetTime(&hrtc, &timeStruct, RTC_FORMAT_BIN); // Read time first!
    HAL_RTC_GetDate(&hrtc, &dateStruct, RTC_FORMAT_BIN);

    // Setup a tm structure based on the RTC
    timeinfo.tm_wday = dateStruct.WeekDay;
    timeinfo.tm_mon  = dateStruct.Month - 1;
    timeinfo.tm_mday = dateStruct.Date;
    timeinfo.tm_year = dateStruct.Year + 100;
    timeinfo.tm_hour = timeStruct.Hours;
    timeinfo.tm_min  = timeStruct.Minutes;
    timeinfo.tm_sec  = timeStruct.Seconds;
    timeinfo.tm_ssec = timeStruct.SubSeconds;

    // Convert to timestamp
    time_t cur_time = mktime(&timeinfo);

    return cur_time;
}

// Convert Date/Time structures to epoch time
uint32_t RTC_ToEpoch()
{
	uint8_t  a;
	uint16_t y;
	uint8_t  m;
	uint32_t JDN;
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	RTC_GetDateTime(&time, &date);

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	// Calculate some coefficients
	a = (14 - date.Month) / 12;
	y = (date.Year + 2000) + 4800 - a; // years since 1 March, 4801 BC
	m = date.Month + (12 * a) - 3; // since 1 March, 4801 BC

	// Gregorian calendar date compute
    JDN  = date.Date;
    JDN += (153 * m + 2) / 5;
    JDN += 365 * y;
    JDN += y / 4;
    JDN += -y / 100;
    JDN += y / 400;
    JDN  = JDN - 32045;
    JDN  = JDN - JULIAN_DATE_BASE;    // Calculate from base date
    JDN *= 86400;                     // Days to seconds
    JDN += time.Hours * 3600;    // ... and today seconds
    JDN += time.Minutes * 60;
    JDN += time.Seconds;

	return JDN;
}

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
	uint32_t tm;
	uint32_t t1;
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t e;
	uint32_t m;
	int16_t  year  = 0;
	int16_t  month = 0;
	int16_t  dow   = 0;
	int16_t  mday  = 0;
	int16_t  hour  = 0;
	int16_t  min   = 0;
	int16_t  sec   = 0;
	uint64_t JD    = 0;
	uint64_t JDN   = 0;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	JD  = ((epoch + 43200) / (86400 >>1 )) + (2440587 << 1) + 1;
	JDN = JD >> 1;

    tm = epoch; t1 = tm / 60; sec  = tm - (t1 * 60);
    tm = t1;    t1 = tm / 60; min  = tm - (t1 * 60);
    tm = t1;    t1 = tm / 24; hour = tm - (t1 * 24);

    dow   = JDN % 7;
    a     = JDN + 32044;
    b     = ((4 * a) + 3) / 146097;
    c     = a - ((146097 * b) / 4);
    d     = ((4 * c) + 3) / 1461;
    e     = c - ((1461 * d) / 4);
    m     = ((5 * e) + 2) / 153;
    mday  = e - (((153 * m) + 2) / 5) + 1;
    month = m + 3 - (12 * (m / 10));
    year  = (100 * b) + d - 4800 + (m / 10);

    date->Year    = year - 2000;
    date->Month   = month;
    date->Date    = mday;
    date->WeekDay = dow;
    time->Hours   = hour;
    time->Minutes = min;
    time->Seconds = sec;
}

