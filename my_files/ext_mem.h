#ifndef _EXT_MEM_H
#define _EXT_MEM_H

#include "my_gpio.h"
#include "cy15b104qn_spi.h"

#define SINGLE_MEM_VOL_BYTES (pow(2, 19))
#define SINGLE_MEM_VOL_FRAMES (SINGLE_MEM_VOL_BYTES/128)
#define CY15104_MEM_NUM 4
#define AVAILABLE_MEM_VOL_FRAMES (SINGLE_MEM_VOL_FRAMES - CY15104_MEM_NUM)



#define PART_ISS 0
#define PART_DCR 1
#define PART_NUM 2 // 0 - ISS, 1 - DCR
//относительный объем памяти выделенной под разные приборы высчитывается как: Vol_1(Fr) = AVAILABLE_MEM_VOL_FRAMES * (Vol_1_rel / (Vol_1_rel + Vol_2_rel))
#define PART_ISS_VOL_REL 256
#define PART_DCR_VOL_REL 32

/** 
  * @brief  структура хранения настроек и параметров внешней памяти
  */
typedef struct
{
  type_CY15B104QN_CONTROL cy15b204[CY15104_MEM_NUM];
	uint16_t fill_part_volume_prc[PART_NUM];
  uint32_t abs_part_volume_frame[PART_NUM];
  uint32_t write_ptr[PART_NUM], read_ptr[PART_NUM];
} type_MEM_CONTROL;


#endif
