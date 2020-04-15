/**
  ******************************************************************************
  * @file           : pn_dcr.c
  * @version        : v1.0
  * @brief          : програмная модель для управления ПН ДеКоР
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn_dcr.h"
#include <stdio.h>

/**
  * @brief  инийиализация полезной нагрузки ДекОр
  * @param  pn_dcr_ptr: указатель на структуру управления ПН1.1
  * @param  uart_ptr: указатель на структуру управления UART через HAL CubeMX
  * @param  mcu_pwr_ch_ptr: указатель на структуру управления питанием части Декор с MCU
  * @param  msr_pwr_ch_ptr: указатель на структуру управления питанием измерительной части Декор
  * @param  num: номер ПН1.1: 0-А, 1-B
  */
void pn_dcr_init(type_PN_DCR_model* pn_dcr_ptr, UART_HandleTypeDef *uart_ptr, type_PWR_CHANNEL* mcu_pwr_ch_ptr, type_PWR_CHANNEL* msr_pwr_ch_ptr)
{
	// установка канала управления питанием
	pn_dcr_ptr->mcu_pwr_ch = mcu_pwr_ch_ptr;
	pn_dcr_ptr->msr_pwr_ch = msr_pwr_ch_ptr;
	// инициализация интерфейса общения
	pn_dcr_uart_init(&pn_dcr_ptr->uart, uart_ptr);
	// сброс ошибок и счетчиков ошибок
	pn_dcr_reset_state(pn_dcr_ptr);
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_reset_state(type_PN_DCR_model* pn_dcr_ptr)
{
	pn_dcr_ptr->status = 0;
	pn_dcr_ptr->rx_frames_cnt = 0;
	pn_dcr_ptr->rx_status_cnt = 0;
	_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_NO_ERROR);
}

/**
  * @brief  создание отчета по работе ПН_ДКР
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_report_create(type_PN_DCR_model* pn_dcr_ptr)
{
	//
	memset((uint8_t*)&pn_dcr_ptr->report, 0xFE, sizeof(type_PNDCR_report));
	//
	pn_dcr_ptr->report.status 					= pn_dcr_ptr->status;
	pn_dcr_ptr->report.error_flags 			= pn_dcr_ptr->error_flags;
	pn_dcr_ptr->report.err_cnt 					= pn_dcr_ptr->error_cnt;
	pn_dcr_ptr->report.outputs 					= pn_dcr_get_outputs_state(pn_dcr_ptr);
	pn_dcr_ptr->report.voltage_mcu 			= pn_dcr_ptr->mcu_pwr_ch->ina226.voltage;
	pn_dcr_ptr->report.current_mcu 			= pn_dcr_ptr->mcu_pwr_ch->ina226.current;
	pn_dcr_ptr->report.voltage_msr 			= pn_dcr_ptr->msr_pwr_ch->ina226.voltage;
	pn_dcr_ptr->report.current_msr 			= pn_dcr_ptr->msr_pwr_ch->ina226.current;
	pn_dcr_ptr->report.uart_rx_cnt 			= pn_dcr_ptr->uart.rx_cnt;
	pn_dcr_ptr->report.uart_tx_cnt 			= pn_dcr_ptr->uart.tx_cnt;
	pn_dcr_ptr->report.rsrv 						= 0xFEFE;
}

/**
  * @brief  чтение состояния выходов ПН ДКР
	* @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  * @retval статус выходов ПН: 0-2 ENA[0..2] для pwr_ch_mcu, 4-6 ENA[0..2] для pwr_ch_msr 
  */
uint8_t pn_dcr_get_outputs_state(type_PN_DCR_model* pn_dcr_ptr)
{
	uint8_t state = 0;
	//
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[0]) & 0x01) << 0;
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[1]) & 0x01) << 1;
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[2]) & 0x01) << 2;
	//
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[0]) & 0x01) << 4;
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[1]) & 0x01) << 5;
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[2]) & 0x01) << 6;
	return state;
}

/**
  * @brief  включение питания ПН1.1
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  mode: тип канала питания согласно #define в .h файле
  */
void pn_dcr_pwr_on(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode)
{
	switch(mode){
		case PN_DCR_PWR_MCU:
			pwr_ch_on_off(pn_dcr_ptr->mcu_pwr_ch, 0x01);
			break;
		case PN_DCR_PWR_MSR:
			pwr_ch_on_off(pn_dcr_ptr->msr_pwr_ch, 0x01);
			break;
		case PN_DCR_PWR_ALL:
			pwr_ch_on_off(pn_dcr_ptr->mcu_pwr_ch, 0x01);
			pwr_ch_on_off(pn_dcr_ptr->msr_pwr_ch, 0x01);
			break;
	}
}

/**
  * @brief  отключение питания ПН1.1
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  mode: тип канала питания согласно #define в .h файле
  */
void pn_dcr_pwr_off(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode)
{
	switch(mode){
		case PN_DCR_PWR_MCU:
			pwr_ch_on_off(pn_dcr_ptr->mcu_pwr_ch, 0x00);
			break;
		case PN_DCR_PWR_MSR:
			pwr_ch_on_off(pn_dcr_ptr->msr_pwr_ch, 0x00);
			break;
		case PN_DCR_PWR_ALL:
			pwr_ch_on_off(pn_dcr_ptr->mcu_pwr_ch, 0x00);
			pwr_ch_on_off(pn_dcr_ptr->msr_pwr_ch, 0x00);
			break;
	}
}

/**
  * @brief  отправка команды в ДеКоР
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  cmd_type: тип команды на отправку #define в .h файле
	* @param  data: данные для передачи в команде
  */
void pn_dcr_send_cmd(type_PN_DCR_model* pn_dcr_ptr, uint8_t cmd_type, uint8_t *data)
{
	type_PN_DCR_frame frame;
	uint8_t len;
	switch(cmd_type){
		case PN_DCR_CMD_GET_TM_STATUS:
			len = _pn_dcr_form_frame(PN_DCR_UART_FRAME_SHORT, &frame, 0x72, 0xC7, data);
			pn_dcr_uart_send(&pn_dcr_ptr->uart, (uint8_t*)&frame, len);
			break;
		case PN_DCR_CMD_GET_DATA_MONITOR:
			len = _pn_dcr_form_frame(PN_DCR_UART_FRAME_SHORT, &frame, 0x72, 0xC1, data);
			pn_dcr_uart_send(&pn_dcr_ptr->uart, (uint8_t*)&frame, len);
			break;
		case PN_DCR_CMD_GET_DATA_MASSIVE:
			len = _pn_dcr_form_frame(PN_DCR_UART_FRAME_SHORT, &frame, 0x72, 0xC3, data);
			pn_dcr_uart_send(&pn_dcr_ptr->uart, (uint8_t*)&frame, len);
			break;
	}
}

/**
  * @brief  проверка наличия данных, принятых от ДеКоР
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  data: данные для передачи в команде
	* @retval длина принятых данных
  */
uint8_t pn_dcr_get_data(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data)
{
	uint8_t leng = 0;
	if (pn_dcr_ptr->uart.rx_len){
		leng = pn_dcr_ptr->uart.rx_len;
		memcpy(data, pn_dcr_ptr->uart.rx_data, pn_dcr_ptr->uart.rx_len);
		pn_dcr_ptr->uart.rx_len = 0;
	}
	return leng;
}

/**
  * @brief  обработка пришедших данных один раз в 10 мс
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  data: данные для передачи в команде
	* @retval длина принятых данных
  */
void pn_dcr_process_rx_frames_10ms(type_PN_DCR_model* pn_dcr_ptr) 
{	
	uint8_t data[128] = {0}, leng;
	//формирование кадров для последующей обработки и их сортировака по типам
	leng = pn_dcr_get_data(pn_dcr_ptr, data);
	if (leng > 0){
		if (leng <= 116){
			memcpy(pn_dcr_ptr->last_received_status, data, leng);
			pn_dcr_ptr->last_received_status_leng = leng;
			pn_dcr_ptr->rx_status_cnt++;
		}
		else if ((leng > 116) && (leng <= 124)){
			memcpy(pn_dcr_ptr->last_received_frame, data, leng);
			pn_dcr_ptr->last_received_frame_leng = leng;
			pn_dcr_ptr->rx_frames_cnt++;
		}
		else{  //если, по какой-то причине пришло больше чем возможно, сохраняем возможную часть и выставляем ошибку
			memcpy(pn_dcr_ptr->last_received_frame, data, 124);
			_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_WRONG_FRAME_LENG);
		}
	}
}

/**
  * @brief  запрос последнего кадра данных для сохранения в память и выкладывания в рашаренную память
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @retval длина данных для сохранения
  */
uint8_t pn_dcr_get_last_frame(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data) 
{	
	uint8_t leng = 0;
	if (pn_dcr_ptr->last_received_frame_leng) {
		memcpy(data, pn_dcr_ptr->last_received_frame, pn_dcr_ptr->last_received_frame_leng);
		leng = pn_dcr_ptr->last_received_frame_leng;
		pn_dcr_ptr->last_received_frame_leng = 0;
		return leng;
	}
	return 0;
}

/**
  * @brief  запрос последнего статуса для выкладывания в рашаренную память
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @retval длина данных для сохранения
  */
uint8_t pn_dcr_get_last_status(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data) 
{
	uint8_t leng = 0;
	if (pn_dcr_ptr->last_received_status_leng){
		memcpy(data, pn_dcr_ptr->last_received_status, pn_dcr_ptr->last_received_status_leng);
		leng = pn_dcr_ptr->last_received_status_leng;
		pn_dcr_ptr->last_received_status_leng = 0;
		return leng;
	}
	return 0;
}

/**
  * @brief  обработка полетного задания  один раз в 10 мс
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_set_flight_task(type_PN_DCR_model* pn_dcr_ptr, uint8_t* mode)
{
	return;
}

/**
  * @brief  обработка полетного задания  один раз в 10 мс
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_process_flight_task_100ms(type_PN_DCR_model* pn_dcr_ptr) 
{
	return;
}

/**
  * @brief  формирование шаблона команды
	* @param  frame_type: тип кадара (короткий/длинный) на отправку #define PN_DCR_UART_FRAME_... в .h файле
  * @param  frame: указатель на структуру кадра в зависимости от frame_type
	* @param  cmd_type: тип команды (0х62, 0x72, 0x40, 0x7A)
	* @param  cmd_code: код команды согласно описанию на ДеКоР
	* @param  data: данные для передачи
	* @retval возвращает длину для передачи данных
	*/
uint8_t _pn_dcr_form_frame(uint8_t frame_type, type_PN_DCR_frame *frame, uint8_t cmd_type, uint8_t cmd_header, uint8_t *data)
{
	switch(frame_type){
		case PN_DCR_UART_FRAME_SHORT:
			frame->shrt.start_header = 'D';
			frame->shrt.cmd_type = cmd_type;
			frame->shrt.cmd_code = cmd_header;
			memcpy(frame->shrt.data, data, 6);
			frame->shrt.stop_tail[0] = ';';
			frame->shrt.stop_tail[1] = '\r';
			frame->shrt.stop_tail[2] = '\n';
			return 12;
		case PN_DCR_UART_FRAME_LONG:
			return 124;
	}
	return 0;
}

/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  * @param  error: ошибка, согласно define-ам PN_DCR_UART_ERR_... в .h
  */
void  _pn_dcr_error_collector(type_PN_DCR_model* pn_dcr_ptr, uint16_t error)
{
  switch(error){
    case PN_DCR_ERR_NO_ERROR:
      pn_dcr_ptr->error_flags = PN_DCR_UART_ERR_NO_ERROR;
      pn_dcr_ptr->error_cnt = 0;
      break;
    case PN_DCR_UART_ERR_BAD_FRAME:
			pn_dcr_ptr->error_flags = error;
      pn_dcr_ptr->error_cnt += 1;
      break;
    default:
      pn_dcr_ptr->error_flags |= PN_DCR_UART_ERR_OTHER;
      pn_dcr_ptr->error_cnt += 1;
      break;
  }
}

//*** Протокол общения с ДеКоР ***//

/**
  * @brief  инициализация интерфейса общения для ПН ДКР
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  mode: тип канала питания согласно #define в .h файле
  */
void pn_dcr_uart_init(type_PNDCR_interface *int_ptr, UART_HandleTypeDef *uart_ptr)
{
	int_ptr->huart = uart_ptr;
	// обнуляем переменные состояний
	int_ptr->tx_len = 0;
	int_ptr->rx_ptr = 0;
	int_ptr->rx_len = 0;
	//запускаем первое чтение байта
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_buff + int_ptr->rx_ptr, 1);
	// чистим буферы приема на всякий случай
	memset(int_ptr->tx_data, 0x00, sizeof(int_ptr->tx_data));
	memset(int_ptr->rx_data, 0x00, sizeof(int_ptr->rx_data));
	memset(int_ptr->rx_buff, 0x00, sizeof(int_ptr->rx_buff));
	//сбрасываем ошибки и счетчики
	_pn_dcr_uart_error_collector(int_ptr, PN_DCR_UART_ERR_NO_ERROR);
}

/**
  * @brief  инициализация интерфейса общения для ПН ДКР
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  mode: тип канала питания согласно #define в .h файле
  */
void pn_dcr_uart_send(type_PNDCR_interface *int_ptr, uint8_t* data, uint8_t len)
{
	memcpy(int_ptr->tx_data, data, len);
	int_ptr->tx_len = len;
	HAL_UART_Transmit_IT(int_ptr->huart, int_ptr->tx_data, int_ptr->tx_len);
}

/**
  * @brief  обработка колбэка на прием данных
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  * @param  mem_ptr: указатлье на внешнюю энергонезависимую память
  */
void pn_dcr_uart_rx_prcs_cb(type_PNDCR_interface *int_ptr)
{
	char frame_end[3] = {';', '\r', '\n'};
	// инкрементация длины
	int_ptr->rx_ptr++;
	// проверка на окончание пакета
	if (int_ptr->rx_ptr >= 3){
		if (memcmp(&int_ptr->rx_buff[int_ptr->rx_ptr-3], frame_end, 3) == 0){
			int_ptr->rx_len = int_ptr->rx_ptr;
			int_ptr->rx_ptr = 0;
			memcpy(int_ptr->rx_data, int_ptr->rx_buff, int_ptr->rx_len);
			int_ptr->rx_cnt += 1; 
		}
		else if (int_ptr->rx_ptr >= 124) {
			int_ptr->rx_len = int_ptr->rx_ptr;
			int_ptr->rx_ptr = 0;
			memcpy(int_ptr->rx_data, int_ptr->rx_buff, int_ptr->rx_len);
			_pn_dcr_uart_error_collector(int_ptr, PN_DCR_UART_ERR_BAD_FRAME);
		}
	}
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_buff + int_ptr->rx_ptr, 1);
}

/**
  * @brief  обработка колбэка на успешную отправку данных данных
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_uart_tx_prcs_cb(type_PNDCR_interface *int_ptr)
{
	int_ptr->tx_cnt++;
}

/**
  * @brief  обработка колбэка на ошибку при приеме или передачи
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_uart_err_prcs_cb(type_PNDCR_interface *int_ptr)
{
	uint32_t HAL_UART_ERROR;
	HAL_UART_ERROR = HAL_UART_GetError(int_ptr->huart);
	if (HAL_UART_ERROR & HAL_UART_ERROR_NE)
	{
		_pn_dcr_uart_error_collector(int_ptr, PN_DCR_UART_ERR_HAL_NOISE);
		HAL_UART_ERROR &= ~HAL_UART_ERROR_NE;
	}
	else if (HAL_UART_ERROR){
		_pn_dcr_uart_error_collector(int_ptr, PN_DCR_UART_ERR_HAL_OTHER);
	}
}

/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
  * @param  error: ошибка, согласно define-ам PN_DCR_UART_ERR_... в .h
  */
void  _pn_dcr_uart_error_collector(type_PNDCR_interface *int_ptr, uint16_t error)
{
  switch(error){
    case PN_DCR_UART_ERR_NO_ERROR:
      int_ptr->error_flags = PN_DCR_UART_ERR_NO_ERROR;
      int_ptr->error_cnt = 0;
      break;
    case PN_DCR_UART_ERR_BAD_FRAME:
    case PN_DCR_UART_ERR_HAL_NOISE:
    case PN_DCR_UART_ERR_HAL_OTHER:
			int_ptr->error_flags = error;
      int_ptr->error_cnt += 1;
      break;
    default:
      int_ptr->error_flags |= PN_DCR_UART_ERR_OTHER;
      int_ptr->error_cnt += 1;
      break;
  }
}

