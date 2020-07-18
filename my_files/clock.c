  /**
  ******************************************************************************
  * @file           : clock.c
  * @version        : v1.0
  * @brief          : надстройка над HAL для работы с RTC
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "clock.h"

/**
  * @brief  устанавливаем время в ноль при перезагрузке
  */
void clock_init(void)
{
  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  //
  sTime.Seconds = 0;
  sTime.Minutes = 0;
  sTime.Hours = 0;
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
  //
  sDate.Year = 0;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
	sDate.WeekDay = 1;
  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
}

/**
  * @brief  получение количества секнду с 2000-года
  */
uint32_t clock_get_time_s(void)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  uint32_t seconds, days, d_years, d_month;
  //
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  //
  d_years = (date.Year)*365 + (date.Year+3)/4 - (date.Year+99)/100 + (date.Year+399)/400;
  d_month = _get_month_offset(date.Year, date.Month);
  days = (date.Date - 1) + d_month + d_years;
  //
  seconds = days*86400 + time.Hours*3600 + time.Minutes*60 + time.Seconds;
	return seconds;
}

/**
  * @brief  получение количества секнду с 1970-г (UNIX-time)
  */
uint32_t clock_get_unix_time_s(void)
{
  type_tm tm_time;
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  uint32_t seconds;
  //
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
  //
  tm_time.tm_year = CLOCK_START_YEAR + date.Year;
  tm_time.tm_mon = date.Month;
  tm_time.tm_mday = date.Date;
  tm_time.tm_hour = time.Hours;
  tm_time.tm_min = time.Minutes;
  tm_time.tm_sec = time.Seconds;
  //
  seconds = xtmtot(&tm_time);
  //
	return seconds;
}

/**
  * @brief  установка времени в HAL_RTC
  * @param  time_s: время в секундах от 2000 года
  */
void clock_set_time_s(uint32_t time_s)
{
  RTC_TimeTypeDef time = {0};
  RTC_DateTypeDef date = {0};
  uint32_t years = 0, days = 0, days_remain = 0, month = 0;
  uint16_t years_d_offset[100] = {0};
  //
  days = time_s / 86400;
  //
  for(uint8_t y=0; y<100; y++){
      years_d_offset[y] = y*365 + (y+3)/4 - (y+99)/100 + (y+399)/400;
      if (y > 0){
          if (days < years_d_offset[y]){
              years = (y-1);
              days_remain = days - years_d_offset[y-1];
              break;
          }
      }
  }
  //
  for(uint8_t m=1; m<=12; m++){
    if (days_remain < _get_month_offset(years, m+1)){
        month = m;
        days_remain = days_remain - _get_month_offset(years, m);
        break;
    }
  }
  //
  date.Year = years;
  date.Month = month;
  date.Date = days_remain + 1;
  date.WeekDay = 1;
  //
  time.Hours = (time_s % 86400) / 3600;
  time.Minutes =  ((time_s % 86400) % 3600) / 60;
  time.Seconds =  ((time_s % 86400) % 3600) % 60;
  //
  HAL_RTC_SetTime(&hrtc, &time, RTC_FORMAT_BIN);
  HAL_RTC_SetDate(&hrtc, &date, RTC_FORMAT_BIN);
}

uint16_t _get_month_offset(uint8_t year, uint8_t month)
{
    const uint16_t month_d_offset[12] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    uint8_t month_d_add = ((year % 4 == 0) && ((year % 100 != 0) && (year % 400 != 0)) && (month>2)) ? 1 : 0;
    return month_d_offset[month-1] + month_d_add;
}

