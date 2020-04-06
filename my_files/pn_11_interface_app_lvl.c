/**
  ******************************************************************************
  * @file           : pn_11_interface_app_lvl.c
  * @version        : v1.0
  * @brief          : реализация упрвня приложения для ПН1.1
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn_11_interface_app_lvl.h"

/**
  * @brief  инийиализация протокола уровня приложения
  * @param  app_lvl_ptr: указатель на структуру управления уровнем приложения
  * @param  huart: указатель на структуру UART
  */
void app_lvl_init(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, UART_HandleTypeDef* huart)
{
	tr_lvl_init(&app_lvl_ptr->tr_lvl, huart);
}

/**
  * @brief  запись данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  addr: адрес в структуре данных APB (байтовый)
  * @param  data: данные для записи по адресу в uint32_t-словах
  * @param  len: длина данных для записи по адресу в uint32_t-словах (максимум 61 слово)
  * @retval 0 - некорректная длина, 1 - успешная попытка передачи
  */
int8_t app_lvl_write(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len)
{
  uint8_t u8_data[256], u8_len = 0;
  if ((len == 0) || (len > 61)) {
    app_lvl_ptr->error |= APP_LVL_DATA_ERROR;
    return 0;
  }
  // очищаем форму на отправку
  memset((uint8_t*)app_lvl_ptr->wr_frame.data, 0x00, sizeof(type_APP_LVL_PCT));
  // формируем управляющий байт
	app_lvl_ptr->wr_frame.ctrl_byte = (APP_LVL_MODE_READ << 6) | ((len-1) & 0x3F);
  // формируем адрес 4-байта
  app_lvl_ptr->wr_frame.addr = addr;
  //заполняем данные: минимальная длина - одно слово
  memcpy((uint8_t*)app_lvl_ptr->wr_frame.data, (uint8_t*)data, 4*len);
  //Отправка данных
  _app_lvl_form_data(&app_lvl_ptr->wr_frame, len, u8_data, &u8_len);
  tr_lvl_send_data(&app_lvl_ptr->tr_lvl, u8_data, u8_len);
  return 1;
}

/**
  * @brief  формирование массива uint8_t правильной эндианности для отправки
  * @param  frame_ptr: указатель на структуру пакета для отправки
  * @param  data_len: длина данных в пакете в uint32_t-cловах
  * @param  u8_data: указатель на массив данных для отправки транспортному уровню
  * @param  u8_len_ptr: длина массива для отправки транспортному уровню
  */
void _app_lvl_form_data(type_APP_LVL_PCT *frame_ptr, uint8_t data_len, uint8_t *u8_data, uint8_t *u8_len_ptr)
{
  u8_data[0] = frame_ptr->ctrl_byte;
  *(uint32_t*)&u8_data[1] = __REV(frame_ptr->ctrl_byte);
  for(uint8_t cnt=0; cnt<data_len; cnt++){
    *(uint32_t*)&u8_data[5] = __REV(frame_ptr->data[cnt]);
  }
  *u8_len_ptr = 1 + 4 + 4*data_len;
}

/**
  * @brief  запрос на чтение данных данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  addr: адрес в структуре данных APB (байтовый)
  * @param  len: длина данных для чтения по адресу в uint32_t-словах (максимум 61 слово)
  * @retval 0 - некорректная длина, 1 - успешная попытка передачи
  */
int8_t app_lvl_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint8_t len)
{
  uint8_t u8_data[256], u8_len = 0;
  if ((len == 0) || (len > 61)) {
    app_lvl_ptr->error |= APP_LVL_DATA_ERROR;
    return 0;
  }
  // очищаем форму на отправку
  memset((uint8_t*)app_lvl_ptr->rd_frame.data, 0x00, sizeof(type_APP_LVL_PCT));
  // формируем управляющий байт
	app_lvl_ptr->rd_frame.ctrl_byte = (APP_LVL_MODE_READ << 6) | ((len-1) & 0x3F);
  // формируем адрес 4-байта
  app_lvl_ptr->rd_frame.addr = addr;
  //Отправка данных
  _app_lvl_form_data(&app_lvl_ptr->rd_frame, 0, u8_data, &u8_len);
  tr_lvl_send_data(&app_lvl_ptr->tr_lvl, u8_data, u8_len);
  return 1;
}

/**
  * @brief  команда на чтение данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @retval  status: 1 - чтение удалось, данные в приемном буфере управляющей структуры, 0 - чтение в ожидании ответа, <0 - ошибка чтения
  */
int8_t app_lvl_read_check(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr)
{
  uint8_t u8_buff[256] = {0}, u8_len = 0, cnt = 0;
  u8_len = rx_data_get(&app_lvl_ptr->tr_lvl, u8_buff);
  for(cnt=0; cnt < (u8_len/4); cnt++){
    app_lvl_ptr->rd_frame.data[cnt] = __REV(*(uint32_t*)&u8_buff[cnt*4]);
  }
	return cnt;
}



