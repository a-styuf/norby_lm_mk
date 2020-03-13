#ifndef _EXT_MEM_H
#define _EXT_MEM_H

#include "my_gpio.h"

#define SINGLE_MEM_VOL_BYTES (pow(2, 19))
#define SINGLE_MEM_VOL_FRAMES (SINGLE_MEM_VOL_BYTES/128)

/** 
  * @brief  структура хранения настроек и параметров внешней памяти
  */
typedef struct
{
	uint8_t fill_volume_iss, fill_volume_dcr;
} type_MEM_CONTROL;

#endif
