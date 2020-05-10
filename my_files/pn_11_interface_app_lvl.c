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
  app_lvl_ptr->rx_timeout = 0;
}

/**
  * @brief  запись данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  addr: адрес в структуре данных APB (байтовый)
  * @param  data: данные для записи по адресу в uint32_t-словах
  * @param  len: длина данных для записи по адресу в uint32_t-словах (максимум APP_LVL_MAX_U32_DATA слов из-за ограничения передачи через CAN)
  * @retval 0 - некорректная длина, 1 - успешная попытка передачи
  */
int8_t app_lvl_write(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len)
{
  uint8_t u8_data[256], u8_len = 0;
  if ((len == 0) || (len > APP_LVL_MAX_U32_DATA)) {
    _app_lvl_error_collector(app_lvl_ptr, APP_LVL_ADDR_ERROR);
    return 0;
  }
  // очищаем форму на отправку
  memset((uint8_t*)&app_lvl_ptr->wr_frame, 0x00, sizeof(type_APP_LVL_PCT));
  // формируем управляющий байт
	app_lvl_ptr->wr_frame.ctrl_byte = (APP_LVL_MODE_WRITE << 6) | ((len-1) & 0x3F);
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
  u8_data[0] = (frame_ptr->ctrl_byte) & 0xFF;
  *(uint32_t*)&u8_data[1] = __REV(frame_ptr->addr);
  for(uint8_t cnt=0; cnt<data_len; cnt++){
    *(uint32_t*)&u8_data[5] = __REV(frame_ptr->data[cnt]);
  }
  *u8_len_ptr = 1 + 4 + 4*data_len;
}

/**
  * @brief  запрос на чтение данных данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  addr: адрес в структуре данных APB (байтовый)
  * @param  len: длина данных для чтения по адресу в uint32_t-словах (максимум APP_LVL_MAX_U32_DATA слов из-за ограничения передачи через CAN)
  * @retval 0 - некорректная длина, 1 - успешная попытка передачи
  */
int8_t app_lvl_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint8_t len)
{
  uint8_t u8_data[256], u8_len = 0;
  if ((len == 0) || (len > APP_LVL_MAX_U32_DATA)) {
    _app_lvl_error_collector(app_lvl_ptr, APP_LVL_ADDR_ERROR);
    return 0;
  }
  // очищаем форму на отправку
  memset((uint8_t*)&app_lvl_ptr->rd_frame, 0x00, sizeof(type_APP_LVL_PCT));
  // формируем управляющий байт
	app_lvl_ptr->rd_frame.ctrl_byte = (APP_LVL_MODE_READ << 6) | ((len-1) & 0x3F);
  // формируем адрес 4-байта
  app_lvl_ptr->rd_frame.addr = addr;
  // снимаем флаг готовности данных
  app_lvl_ptr->rx_valid_flag = 0; // данные приняты и готовы для чтения
  app_lvl_ptr->rx_ready_flag = 0; // данные приняты и еще не читались
  app_lvl_ptr->rx_timeout_flag = 1;
  app_lvl_ptr->rx_timeout = APP_LVL_DEFAULT_TIMEOUT_MS;
  //Отправка данных
  _app_lvl_form_data(&app_lvl_ptr->rd_frame, 0, u8_data, &u8_len);
  tr_lvl_send_data(&app_lvl_ptr->tr_lvl, u8_data, u8_len);
  return 1;
}

/**
  * @brief  команда на обработку потока уровня приложения
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void app_lvl_process(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint16_t period_ms)
{
  uint8_t uint32_len = 0;
  // проверяем наличие ответа для процедуры чтения
  if (app_lvl_ptr->rx_timeout != 0){
    uint32_len = app_lvl_read_check(app_lvl_ptr);
    if (uint32_len){
      app_lvl_ptr -> rx_ready_flag = 1;
      app_lvl_ptr -> rx_valid_flag = 1;
      app_lvl_ptr -> rx_timeout = 0;
    }
  }
  // обработка таймаута вохзможна неточность в period_ms
  app_lvl_ptr->rx_timeout -= period_ms;
  if ((app_lvl_ptr->rx_timeout == 0) || (app_lvl_ptr->rx_timeout >= APP_LVL_DEFAULT_TIMEOUT_MS)){
    app_lvl_ptr->rx_timeout = 0;
    if ((app_lvl_ptr->rx_ready_flag == 0) && (app_lvl_ptr->rx_timeout_flag == 1)){
      _app_lvl_error_collector(app_lvl_ptr, APP_LVL_TIMEOUT_ERROR);
    }
    app_lvl_ptr->rx_timeout_flag = 0;
  }
}

/**
  * @brief  команда вкобчающая обработку приема, а так же организующая 
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void app_lvl_process_and_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint16_t period_ms)
{
  //тут вызываем обработку, приема для уровня приложения
  app_lvl_process(app_lvl_ptr, period_ms);
  //
}

/**
  * @brief  команда на чтение данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @retval  status: >0 - счетчик принятых данных в uint32_t словах, 0 - чтение в ожидании ответа, <0 - ошибка чтения
  */
int8_t app_lvl_read_check(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr)
{
  uint8_t u8_buff[256] = {0}, u8_len = 0, cnt = 0;
  u8_len = rx_data_get(&app_lvl_ptr->tr_lvl, u8_buff);
	if (u8_len){
		for(cnt=0; cnt < (u8_len/4); cnt++){
			app_lvl_ptr->rd_frame.data[cnt] = __REV(*(uint32_t*)&u8_buff[cnt*4]);
		}
		return cnt;
	}
  return 0;
}

/**
  * @brief  команда на чтение последнего принятого пакета
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  last_data: последний принятый пакет (не более 128 байт, остальное обрежится), в формате type_APP_LVL_PCT
  * @retval  status: >0 - длина свежих данных, 0 - данные уже были прочитаны или нетданных
  */
uint8_t app_lvl_get_last_rx_frame(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint8_t *last_data)
{
  if (app_lvl_ptr->rx_ready_flag){
    memcpy(last_data, (uint8_t*)&app_lvl_ptr->rd_frame, 128);
    app_lvl_ptr->rx_ready_flag = 0;
    return 128;
  }
	return 0;
}

/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  app_lvl_ptr: указатель на структуру управления УРОВНЕМ ПРИЛОЖЕНИЯ
  * @param  error: ошибка, согласно define-ам APP_LVL_... в .h
  */
void  _app_lvl_error_collector(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint16_t error)
{
  switch(error){
    case APP_LVL_NO_ERR:
      app_lvl_ptr->error_flags = APP_LVL_NO_ERR;
      app_lvl_ptr->error_cnt = 0;
      break;
    case APP_LVL_ADDR_ERROR:
    case APP_LVL_DATA_ERROR:
    case APP_LVL_TIMEOUT_ERROR:
			app_lvl_ptr->error_flags |= error;
      app_lvl_ptr->error_cnt += 1;
      break;
    default:
      app_lvl_ptr->error_flags |= APP_LVL_OTHER_ERROR;
      app_lvl_ptr->error_cnt += 1;
      break;
  }
}
