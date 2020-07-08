#ifndef _EXT_MEM_H
#define _EXT_MEM_H

#include "cy15b104qn_spi.h"
#include "my_gpio.h"
#include "lm_interfaces_data.h"
#include "crc16.h"
#include "critical_lock.h"
#include "iwdg.h"

#define CY15B104_MEM_NUM 4  // кол-во микросхем памяти
#define SINGLE_MEM_VOL_FRAMES (CY15_VOLUME_BYTES/128)  // кол-во кадров в отдельной микросхеме памяти
#define FULL_MEM_VOL_FRAMES (SINGLE_MEM_VOL_FRAMES*CY15B104_MEM_NUM)  // общее количество кадров по 128-байт в физической памяти
#define FRAME_MEM_VOL_FRAMES (SINGLE_MEM_VOL_FRAMES*CY15B104_MEM_NUM - CY15B104_MEM_NUM)  // количество кадров в физической памяти за исключением кадров для параметров

#define PART_ISS 0
#define PART_DCR 1
#define PART_DCR_FLIGHT_TASK_1 2
#define PART_DCR_FLIGHT_TASK_2 3
#define PART_DCR_STATUS 4
#define PART_NUM 5 // 0 - ISS, 1 - DCR, 2 - DCR Flight task, 2 - DCR Flight task, 4 - DCR status mem

#define PART_ALL_MEM 127  // переменная для определения всей памяти

// есть два типа памяти - постоянного размера (например для полетного задания)

// Постоянная память: маппирование в конце физической памяти, по абсолютному размеру\
// размер в блоках по 128-байт
#define PART_DCR_FLIGHT_TASK_1_CONST      16
#define PART_DCR_FLIGHT_TASK_2_CONST      16
#define PART_DCR_STATUS_CONST           16
#define PART_FULL_CONST                 48 //общий размер памяти выделенной под постоянные блоки в 128-байтовых блоков
// Относительная память: маппирование частей памяти распределяемой относительно: сумма частей не должна превышать (полную память - постоянная часть)
// относительный объем памяти выделенной под разные приборы высчитывается как: Vol_1(Fr) = AVLL_MEM_VOL_FRAMES * (Vol_1_rel / (PART_FULL_VOL_REL))
#define PART_FULL_VOL_REL               256
#define PART_ISS_VOL_REL                192
#define PART_DCR_VOL_REL                64

//
#define PART_MODE_READ_BLOCK           0x00  // указатель чтения доганяет указатель записи и блокается
#define PART_MODE_WRITE_BLOCK          0x01  // указатель записи доганяет указатель чтения и блокается
#define PART_MODE_REWRITE              0x02  // указатель записи независит от указателя чтения
#define PART_MODE_SMART_COIL_WRITE     0x03  // указатель записи толкает указатель чтения в случае достижения оного, указатель чтения блокируется при достижении указателя записи

#define MODE_READ              0x01
#define MODE_WRITE             0x02

/** 
  * @brief  структура хранения настроек отдельной части памяти
  */
typedef struct
{
  uint32_t write_ptr, read_ptr;
  uint32_t full_frame_num;
  uint32_t start_frame_num, finish_frame_num;
  uint8_t frame_mode, mode;
} type_MEM_PART_CONTROL;

/** 
  * @brief  структура хранения настроек и параметров внешней памяти
  */
typedef struct
{
  type_CY15B104QN_CONTROL cy15b104[CY15B104_MEM_NUM];
  type_MEM_PART_CONTROL part[PART_NUM];
  uint32_t read_ptr;
} type_MEM_CONTROL;

//
int8_t ext_mem_init(type_MEM_CONTROL* mem_ptr, SPI_HandleTypeDef* spi_ptr);
void ext_mem_any_write(type_MEM_CONTROL* mem_ptr, uint32_t frame_addr, uint8_t* buff);
void ext_mem_any_read(type_MEM_CONTROL* mem_ptr, uint32_t frame_addr, uint8_t* buff);
void ext_mem_any_read_8b_block(type_MEM_CONTROL* mem_ptr, uint32_t frame_addr, uint8_t offset, uint8_t* buff);
void ext_mem_any_line_read(type_MEM_CONTROL* mem_ptr, uint8_t* buff);
void ext_mem_any_line_read_8b_block(type_MEM_CONTROL* mem_ptr, uint8_t offset, uint8_t* buff);
int32_t ext_mem_check(type_MEM_CONTROL* mem_ptr, uint8_t symbol);
int32_t ext_mem_wr_param(type_MEM_CONTROL* mem_ptr, uint8_t *param);
int32_t ext_mem_rd_param(type_MEM_CONTROL* mem_ptr, uint8_t *param);
void ext_mem_wr_data_frame(type_MEM_CONTROL* mem_ptr, uint32_t addr, uint8_t *frame);
void ext_mem_rd_data_frame(type_MEM_CONTROL* mem_ptr, uint32_t addr, uint8_t *frame);
void ext_mem_rd_data_frame_8b_block(type_MEM_CONTROL* mem_ptr, uint32_t addr, uint8_t offset, uint8_t *frame);
void ext_mem_wr_frame_to_part(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t part_num);
void ext_mem_rd_frame_from_part(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t part_num);
void ext_mem_rd_frame_from_part_by_addr(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t fr_addr, uint8_t part_num);
void ext_mem_wr_frame_from_part_by_addr(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t fr_addr, uint8_t part_num);
void ext_mem_full_erase(type_MEM_CONTROL* mem_ptr, uint8_t symbol);
void ext_mem_format_part(type_MEM_CONTROL* mem_ptr, uint8_t part_num);
int8_t ext_mem_set_rd_ptr_for_part(type_MEM_CONTROL* mem_ptr, uint8_t *part_num, uint32_t *rd_ptr);
void ext_mem_read_from_part_8b(type_MEM_CONTROL* mem_ptr, uint8_t offset, uint8_t *frame_part, uint8_t part_num);
//
uint32_t part_rel_init(type_MEM_PART_CONTROL* part_ptr, uint8_t mode, uint16_t full_rel_vol, uint16_t rel_vol, uint32_t start_frame_addr);
uint32_t part_const_init(type_MEM_PART_CONTROL* part_ptr, uint8_t mode, uint16_t const_vol, uint32_t start_frame_addr);
uint8_t part_get_free_volume_in_percantage(type_MEM_PART_CONTROL* part_ptr);
uint8_t part_wr_rd_ptr_calc(type_MEM_PART_CONTROL* part_ptr, uint8_t mode);

#endif
