/**
  ******************************************************************************
  * @file           : ext_mem.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX HAL для работы с внешней SPI памятью
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "ext_mem.h"
#include <stdio.h>

/**
  * @brief  инициализация работы с CY15X104QN: специально использую блокирующий режим
  * @param  mem_ptr: структура для управления памятью
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t ext_mem_init(type_MEM_CONTROL* mem_ptr, SPI_HandleTypeDef* spi_ptr)
{
  int8_t report = 0;
  uint32_t start_addr=0;
  mem_ptr->read_ptr = 0;
  // инициализируем память
  report += cy15_init(&mem_ptr->cy15b104[0], spi_ptr, GPIOD, 10);
  report += cy15_init(&mem_ptr->cy15b104[1], spi_ptr, GPIOD, 11);
  report += cy15_init(&mem_ptr->cy15b104[2], spi_ptr, GPIOD, 12);
	report += cy15_init(&mem_ptr->cy15b104[3], spi_ptr, GPIOD, 13);
  // инициализируем блоки памяти для переферии
  start_addr = part_rel_init(&mem_ptr->part[PART_ISS], PART_MODE_REWRITE, PART_FULL_VOL_REL, PART_ISS_VOL_REL, start_addr);
  start_addr = part_rel_init(&mem_ptr->part[PART_DCR], PART_MODE_REWRITE, PART_FULL_VOL_REL, PART_DCR_VOL_REL, start_addr);
  start_addr = part_const_init(&mem_ptr->part[PART_DCR_FLIGHT_TASK], PART_MODE_REWRITE, PART_DCR_FLIGHT_TASK_CONST, start_addr);
  //
  return report;
}


/**
  * @brief  запись 128-ми байтового блока в произволное место памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  frame_addr: адрес кадра в памяти
  * @param  buff: указатель на блок памяти
  */
void ext_mem_any_write(type_MEM_CONTROL* mem_ptr, uint32_t frame_addr, uint8_t* buff)
{
  uint32_t b_addr=0;
  for(uint8_t i=0; i<CY15B104_MEM_NUM; i++){
    if (((i*SINGLE_MEM_VOL_FRAMES) <= frame_addr) && (frame_addr < ((i+1)*SINGLE_MEM_VOL_FRAMES))){
      b_addr = 128*(frame_addr - (i*SINGLE_MEM_VOL_FRAMES));
      cy15_write(&mem_ptr->cy15b104[i], b_addr, buff, 128);
			break;
    }
  }
}

/**
  * @brief  чтение 128-ми байтового блока из произволного места памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  frame_addr: адрес кадра в памяти
  * @param  buff: указатель на блок памяти
  */
void ext_mem_any_read(type_MEM_CONTROL* mem_ptr, uint32_t frame_addr, uint8_t* buff)
{
  uint32_t b_addr=0;
  for(uint8_t i=0; i<CY15B104_MEM_NUM; i++){
    if (((i*SINGLE_MEM_VOL_FRAMES) <= frame_addr) && (frame_addr < ((i+1)*SINGLE_MEM_VOL_FRAMES))){
      b_addr = 128*(frame_addr - (i*SINGLE_MEM_VOL_FRAMES));
      cy15_read(&mem_ptr->cy15b104[i], b_addr, buff, 128);
			break;
    }
  }
}

/**
  * @brief  последовательнрое чтение 128-ми байтового блока из памяти по read_ptr
  * @param  mem_ptr: структура для управления памятью
  * @param  buff: указатель на блок памяти
  */
void ext_mem_any_line_read(type_MEM_CONTROL* mem_ptr, uint8_t* buff)
{
  uint32_t b_addr=0;
  for(uint8_t i=0; i<CY15B104_MEM_NUM; i++){
    if (((i*SINGLE_MEM_VOL_FRAMES) <= mem_ptr->read_ptr) && (mem_ptr->read_ptr < ((i+1)*SINGLE_MEM_VOL_FRAMES))){
      b_addr = 128*(mem_ptr->read_ptr - (i*SINGLE_MEM_VOL_FRAMES));
      cy15_read(&mem_ptr->cy15b104[i], b_addr, buff, 128);
			break;
    }
  }
  if (mem_ptr->read_ptr >= FULL_MEM_VOL_FRAMES) mem_ptr->read_ptr = 0;
  else mem_ptr->read_ptr += 1;
}


/**
  * @brief  блокирующая неразрушающая проверка памяти 128-ми байтными блоками
  * @param  mem_ptr: структура для управления памятью
  * @param  symbol: символ для проверки
  * @param  retval: -1 - ошибка проверки, остальное - адрес блока с ошибкой 
  */
int32_t ext_mem_check(type_MEM_CONTROL* mem_ptr, uint8_t symbol)
{
  uint8_t buff[128], control_buff[128], read_buf[128];
  memset(control_buff, symbol , 128);
  for(uint32_t addr=0; addr < FULL_MEM_VOL_FRAMES; addr++){
    ext_mem_any_read(mem_ptr, addr, buff);
    ext_mem_any_write(mem_ptr, addr, control_buff);
    ext_mem_any_read(mem_ptr, addr, read_buf);
    ext_mem_any_write(mem_ptr, addr, buff);
    if (memcmp(control_buff, read_buf, 128)) {
      return addr;
    }
  }
  return -1;
}

/**
  * @brief  запись параметров (128-байт)
  * @param  mem_ptr: структура для управления памятью
  * @param  param: указатель на структуру с параметрами
  * @param  retval: 1 - параметры записаны, 0 - ошибка записи 
  */
int32_t ext_mem_wr_param(type_MEM_CONTROL* mem_ptr, uint8_t *param)
{
  uint8_t buff[128];
	uint16_t crc16;
  memcpy(buff, param, 126);
  crc16 = norby_crc16_calc(buff, 126);
  *(uint16_t*)&buff[126] = crc16;
  for(uint8_t num=0; num < CY15B104_MEM_NUM; num++){
    ext_mem_any_write(mem_ptr, num*SINGLE_MEM_VOL_FRAMES, buff);
  }
  return 1;
}

/**
  * @brief  чтение параметров (128-байт)
  * @param  mem_ptr: структура для управления памятью
  * @param  param: указатель на структуру с параметрами
  * @param  retval: 1 - параметры прочитаны, 0 - ошибка чтения 
  */
int32_t ext_mem_rd_param(type_MEM_CONTROL* mem_ptr, uint8_t *param)
{
  uint8_t buff[128];
	uint16_t crc16=0;
  for(uint8_t num=0; num < CY15B104_MEM_NUM; num++){
    ext_mem_any_read(mem_ptr, num*SINGLE_MEM_VOL_FRAMES, buff);
    memcpy(param, buff, 128);
		crc16 = norby_crc16_calc(param, 128);
    if(crc16 == 0){
      return 1;
    }
  }
  return 0;
}

/**
  * @brief  запись кадра с данными
  * @param  mem_ptr: структура для управления памятью
  * @param  addr: номер кадра в пространстве кадров с данными
  * @param  frame: указатель на структуру с кадром
  */
void ext_mem_wr_data_frame(type_MEM_CONTROL* mem_ptr, uint32_t addr, uint8_t *frame)
{
  uint32_t real_addr;
  real_addr = addr + (addr/(SINGLE_MEM_VOL_FRAMES-1)) + 1;
  ext_mem_any_write(mem_ptr, real_addr, frame);
}

/**
  * @brief  чтение кадров с данными
  * @param  mem_ptr: структура для управления памятью
  * @param  addr: номер кадра в пространстве кадров с данными
  * @param  frame: указатель на структуру с кадром
  */
void ext_mem_rd_data_frame(type_MEM_CONTROL* mem_ptr, uint32_t addr, uint8_t *frame)
{
  uint32_t real_addr;
  real_addr = addr + (addr/(SINGLE_MEM_VOL_FRAMES-1)) + 1;
  ext_mem_any_read(mem_ptr, real_addr, frame);
}

/**
  * @brief  запись кадра с данными в определенную область памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  frame: указатель на структуру с кадром
  * @param  part_num: номер тома памяти
  */
void ext_mem_wr_frame_to_part(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t part_num)
{
  part_wr_rd_ptr_calc(&mem_ptr->part[part_num], MODE_WRITE);
  ext_mem_wr_data_frame(mem_ptr, mem_ptr->part[part_num].write_ptr + mem_ptr->part[part_num].start_frame_num, frame);
}

/**
  * @brief  чтение кадра с данными из определенной области памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  frame: указатель на структуру с кадром
  * @param  part_num: номер тома памяти
  */
void ext_mem_rd_frame_from_part(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t part_num)
{
  part_wr_rd_ptr_calc(&mem_ptr->part[part_num], MODE_READ);
  ext_mem_rd_data_frame(mem_ptr, mem_ptr->part[part_num].read_ptr + mem_ptr->part[part_num].start_frame_num, frame);
}

/**
  * @brief  запись кадра в блок по адресу
  * @param  mem_ptr: структура для управления памятью
  * @param  frame: указатель на структуру с кадром
  * @param  fr_addr: адрес кадра в блоке
  * @param  part_num: номер тома памяти
  */
void ext_mem_rd_frame_from_part_by_addr(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t fr_addr, uint8_t part_num)
{
  ext_mem_rd_data_frame(mem_ptr, mem_ptr->part[part_num].start_frame_num + fr_addr, frame);
}

/**
  * @brief  чтение кадра из блока по адресу
  * @param  mem_ptr: структура для управления памятью
  * @param  frame: указатель на структуру с кадром
  * @param  fr_addr: адрес кадра в блоке
  * @param  part_num: номер тома памяти
  */
void ext_mem_wr_frame_from_part_by_addr(type_MEM_CONTROL* mem_ptr, uint8_t *frame, uint8_t fr_addr, uint8_t part_num)
{
  ext_mem_wr_data_frame(mem_ptr, mem_ptr->part[part_num].start_frame_num + fr_addr, frame);
}

/**
  * @brief  блокирующее стирание всей памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  symbol: символ для записи
  */
void ext_mem_full_erase(type_MEM_CONTROL* mem_ptr, uint8_t symbol)
{
  uint8_t buff[128];
  memset(buff, symbol, 128);
  for(uint32_t addr=0; addr < FULL_MEM_VOL_FRAMES; addr++){
    ext_mem_any_write(mem_ptr, addr, buff);
  }
}

/**
  * @brief  форматирование тома памяти, блокирующее
  * @param  mem_ptr: структура для управления памятью
  * @param  part_num: номер тома памяти
  */
void ext_mem_format_part(type_MEM_CONTROL* mem_ptr, uint8_t part_num)
{
  uint8_t frame[128] = {0};
  memset(frame, 0xFE, 128);
  frame[1] = part_num;
  for(uint32_t addr = 0; addr < mem_ptr->part[part_num].full_frame_num; addr++){
    *(uint32_t*)&frame[2] = __REV(addr);
    ext_mem_wr_data_frame(mem_ptr, addr + mem_ptr->part[part_num].start_frame_num, frame);
  }
}

///*** Работа с блоками памяти ***///

/**
  * @brief  инициализация работы с отдельным куском памяти c относительным распределением памяти
  * @param  part_ptr: структура для управления памятью
  * @param  mode: тип чтения-записи в блок памяти
  * @param  full_rel_vol: общий отнсительный объем памяти
  * @param  rel_vol: относительный объем блока памяти
  * @param  start_frame_addr: адрес блока памяти в общем пространстве доступных кадров
  * @retval адрес начала следующего тома области в кадрах
  */
uint32_t part_rel_init(type_MEM_PART_CONTROL* part_ptr, uint8_t mode, uint16_t full_rel_vol, uint16_t rel_vol, uint32_t start_frame_addr)
{
  part_ptr->start_frame_num = start_frame_addr;
  part_ptr->full_frame_num = (1. * (FRAME_MEM_VOL_FRAMES - PART_FULL_CONST) * rel_vol) / full_rel_vol;
  part_ptr->finish_frame_num = part_ptr->start_frame_num + part_ptr->full_frame_num - 1;
  // 
  part_ptr->write_ptr = 0;
  part_ptr->read_ptr = 0;
  part_ptr->mode = mode;
  //
  part_ptr->part_fill_volume_prc = 0;
	//
	return part_ptr->finish_frame_num + 1;
}

/**
  * @brief  инициализация работы с отдельным куском памяти c постояннаым распределением памяти
  * @param  part_ptr: структура для управления памятью
  * @param  mode: тип чтения-записи в блок памяти
  * @param  const_vol:объем блока памяти в 128-байтных кадрах
  * @param  start_frame_addr: адрес блока памяти в общем пространстве доступных кадров
  * @retval адрес начала следующего тома области в кадрах
  */
uint32_t part_const_init(type_MEM_PART_CONTROL* part_ptr, uint8_t mode, uint16_t const_vol, uint32_t start_frame_addr)
{
  // подсчет границ и объема отдельной части
  part_ptr->start_frame_num = start_frame_addr;
  part_ptr->full_frame_num = const_vol;
  part_ptr->finish_frame_num = part_ptr->start_frame_num + part_ptr->full_frame_num - 1;
  // 
  part_ptr->write_ptr = 0;
  part_ptr->read_ptr = 0;
  part_ptr->mode = mode;
  //
  part_ptr->part_fill_volume_prc = 0;
	//
	return part_ptr->finish_frame_num + 1;
}

/**
  * @brief  подсчет оставшейся памяти для отдельной части
  * @param  part_ptr: структура для управления памятью
  * @retval заполненнсть памяти в процентах
  */
uint8_t part_fill_volume(type_MEM_PART_CONTROL* part_ptr)
{
  uint32_t free_frames = 0;
  if (part_ptr->write_ptr < part_ptr->read_ptr){
    free_frames = (part_ptr->read_ptr - part_ptr->write_ptr);
  }
  else{
    free_frames = (part_ptr->write_ptr - part_ptr->read_ptr);
  }
  return (uint8_t)(100.*(part_ptr->full_frame_num - free_frames)/part_ptr->full_frame_num);
}

/**
  * @brief  подсчет поведения указателя записи и чтения при записи
  * @param  part_ptr: указатель на структуру контроля отдельного тома памяти
  * @retval  1-можно писать, запись заблокированна
  */
uint8_t part_wr_rd_ptr_calc(type_MEM_PART_CONTROL* part_ptr, uint8_t mode)
{
  int8_t report = 0;
  switch(part_ptr->mode){
    case PART_MODE_READ_BLOCK:  // указатель чтения доганяет указатель записи и блокается
      if (mode == MODE_WRITE){
        part_ptr->write_ptr += 1;
        if ((part_ptr->write_ptr + part_ptr->start_frame_num) > part_ptr->finish_frame_num) part_ptr->write_ptr = 0;
        report = 1;
      }
      else if (mode == MODE_READ){
        report = 1;
        if(part_ptr->read_ptr == part_ptr->write_ptr) report = 0;
        else part_ptr->read_ptr += 1;
        if ((part_ptr->read_ptr + part_ptr->start_frame_num) > part_ptr->finish_frame_num) part_ptr->read_ptr = 0;
      }
    break;
    case PART_MODE_WRITE_BLOCK:  // указатель записи доганяет указатель чтения и блокается
      if (mode == MODE_WRITE){
        report = 1;
        if(part_ptr->write_ptr == part_ptr->read_ptr) report = 0;
        else part_ptr->write_ptr += 1;
        if ((part_ptr->write_ptr + part_ptr->start_frame_num) > part_ptr->finish_frame_num) part_ptr->write_ptr = 0;
      }
      else if (mode == MODE_READ){
        part_ptr->read_ptr += 1;
        if ((part_ptr->read_ptr + part_ptr->start_frame_num) > part_ptr->finish_frame_num) part_ptr->read_ptr = 0;
        report = 1;
      }
    break;
    case PART_MODE_REWRITE:  // указатель записи независит от указателя чтения
    default: // PART_MODE_REWRITE
      if (mode == MODE_WRITE){
        part_ptr->write_ptr += 1;
        if ((part_ptr->write_ptr + part_ptr->start_frame_num) > part_ptr->finish_frame_num) part_ptr->write_ptr = 0;
        report = 1;
      }
      else if (mode == MODE_READ){
        part_ptr->read_ptr += 1;
        if ((part_ptr->read_ptr + part_ptr->start_frame_num) > part_ptr->finish_frame_num) part_ptr->read_ptr = 0;
        report = 1;
      }
    break;
  }
  return report;
}


