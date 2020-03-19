/**
  ******************************************************************************
  * @file           : ext_mem.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX HAL для работы с внешней SPI памятью
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "ext_mem.h"

/**
  * @brief  инициализация работы с CY15X104QN: специально использую блокирующий режим
  * @param  mem_ptr: структура для управления памятью
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t ext_mem_init(type_MEM_CONTROL* mem_ptr)
{
  return 0;
}

/**
  * @brief  чтение блока из памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  buff: указатель на блок памяти
  * @param  len_ptr: длина данных
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t ext_mem_read(type_MEM_CONTROL* mem_ptr)
{
  
  return 0;
}

/**
  * @brief  запись блока в памяти
  * @param  mem_ptr: структура для управления памятью
  * @param  buff: указатель на блок памяти
  * @param  len_ptr: длина данных
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t ext_mem_write(type_MEM_CONTROL* mem_ptr)
{

  return 0;
}
