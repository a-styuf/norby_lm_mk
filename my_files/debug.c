  /**
  ******************************************************************************
  * @file           : debug.c
  * @version        : v1.0
  * @brief          : функции для отладки, отключаемы define DEBUG
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "debug.h"

/**
  * @brief  вывод времени в printf
  */
void printf_time(void)
{
	RTC_TimeTypeDef time;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	printf("%02d:%02d:%2.3f ", time.Hours, time.Minutes, time.Seconds + time.SubSeconds*(1./(1+time.SecondFraction)));
}
