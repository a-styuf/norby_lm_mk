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
	printf("%02d:%02d:%06.3f ", time.Hours, time.Minutes, time.Seconds + (1 - time.SubSeconds*(1./(1+time.SecondFraction)))); // описании не написано, но похоже SubSeconds идет сверху вниз
}

/**
  * @brief  вывод на экран массива hex-значений
  * @param  buff: указатель на блок памяти
  * @param  len: длина данных
  * @param  end_char: символ окончания вывода
  */
void printf_buff(uint8_t *buff, uint8_t len, char end_char)
{
  for (uint8_t i=0; i<len; i++){
    printf("%02X ", buff[i]);
  }
  printf("%c", end_char);
}
