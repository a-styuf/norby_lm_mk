/**
  ******************************************************************************
  * @file           : pn_dcr.c
  * @version        : v1.0
  * @brief          : програмная модель для управления ПН ДеКоР
	* @note          	: для корректной работы данного блока необходим вызов следующих функций с временами по таймеру
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
	pwr_ch_set_bound(mcu_pwr_ch_ptr, PN_DCR_VOLT_MAX, PN_DCR_VOLT_MIN, PN_DCR_MC_PWR_MAX, PN_DCR_MC_PWR_MIN);
	pn_dcr_ptr->msr_pwr_ch = msr_pwr_ch_ptr;
	pwr_ch_set_bound(msr_pwr_ch_ptr, PN_DCR_VOLT_MAX, PN_DCR_VOLT_MIN, PN_DCR_MSR_PWR_MAX, PN_DCR_MSR_PWR_MIN);
	// инициализация интерфейса общения
	pn_dcr_uart_init(&pn_dcr_ptr->uart, uart_ptr);
	// сброс ошибок и счетчиков ошибок
	pn_dcr_reset_state(pn_dcr_ptr);
	// сброс ошибок и счетчиков ошибок
	pn_dcr_fill_default_flight_task(pn_dcr_ptr);
}

/**
  * @brief  запоняем полетное задание по умолчанию
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_dcr_fill_default_flight_task(type_PN_DCR_model* pn_dcr_ptr)
{
	uint8_t data[8] = {0x72, 0xC7, 0, 0, 0, 0, 0, 0};
	//
	memset((uint8_t*)&pn_dcr_ptr->fl_task.default_flt, 0x00, sizeof(type_PNDCR_FlightTask));
	// Задание циклограммы по умолчанию
	// Включаем питание МК ДеКоР
	_pn_dcr_fill_data_array(data, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	pn_dcr_fill_flight_task_step(&pn_dcr_ptr->fl_task.default_flt.step[0], DCR_PN_FLT_TYPE_PWR, 0x01, 6000, data);
	// Заправшиваем статут
	_pn_dcr_fill_data_array(data, 0x72, 0xC7, 0, 0, 0, 0, 0, 0);
	pn_dcr_fill_flight_task_step(&pn_dcr_ptr->fl_task.default_flt.step[1], DCR_PN_FLT_TYPE_UART, 0x01, 10000, data);
	// Заправшиваем статут
	_pn_dcr_fill_data_array(data, 0x72, 0xC7, 0, 0, 0, 0, 0, 0);
	pn_dcr_fill_flight_task_step(&pn_dcr_ptr->fl_task.default_flt.step[2], DCR_PN_FLT_TYPE_UART, 0x01, 10000, data);
	// Заправшиваем статут
	_pn_dcr_fill_data_array(data, 0x72, 0xC7, 0, 0, 0, 0, 0, 0);
	pn_dcr_fill_flight_task_step(&pn_dcr_ptr->fl_task.default_flt.step[3], DCR_PN_FLT_TYPE_UART, 0x01, 10000, data);
	// Отключаем ДеКоР на 20 секунд
	_pn_dcr_fill_data_array(data, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
	pn_dcr_fill_flight_task_step(&pn_dcr_ptr->fl_task.default_flt.step[4], DCR_PN_FLT_TYPE_PWR, 0x00, 20000, data);
}

/**
  * @brief  создание отдельного шага для полетного задания ДеКоР
  * @param  pn_dcr_ptr: указатель на отдельный шаг полетного задания
  */
void pn_dcr_fill_flight_task_step(type_PNDCR_FlightTask_Step* step, uint8_t type, uint8_t cmd, uint32_t pause_ms, uint8_t *data)
{
	step->type = type;
	step->cmd = cmd;
	step->pause_ms = pause_ms;
	step->rsrv = 0xFEFE;
	memcpy(step->data, data, 8);
}

void _pn_dcr_fill_data_array(uint8_t*data, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7)
{
	data[0] = data_0;
	data[1] = data_1;
	data[2] = data_2;
	data[3] = data_3;
	data[4] = data_4;
	data[5] = data_5;
	data[6] = data_6;
	data[7] = data_7;
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
	pn_dcr_ptr->pwr_check_timeout_ms = PN_DCR_PWR_TIMEOUT_MS;
	_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_NO_ERROR, 0);
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
	// состояние выходов канала питания МК
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[0]) & 0x01) << 0;
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[1]) & 0x01) << 1;
	state |= (gpio_get(&pn_dcr_ptr->mcu_pwr_ch->ena[2]) & 0x01) << 2;
	// состояние выходов измерительного канала питания
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
	pn_dcr_ptr->pwr_check_timeout_ms = PN_DCR_PWR_TIMEOUT_MS;
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
	pn_dcr_ptr->pwr_check_timeout_ms = PN_DCR_PWR_TIMEOUT_MS;
}

/**
  * @brief  проверка параметров питания
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  * @retval  ошибки каналов питания ДеКоР
  */
uint8_t pn_dcr_pwr_check(type_PN_DCR_model* pn_dcr_ptr, uint16_t timeout)
{
	uint8_t report = 0, error1 = 0, error2 = 0;
	if (timeout == 0){
		if (pwr_ch_get_error(pn_dcr_ptr->mcu_pwr_ch, &error1)){
			report = error1;
		}
		if (pwr_ch_get_error(pn_dcr_ptr->msr_pwr_ch, &error2)){
			report = (error2 << 4);
		}
	}
	return report;
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
			_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_WRONG_FRAME_LENG, 0);
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
  * @brief  запрос последнего статуса ПН ДеКоР для выкладывания в рашаренную память
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
  * @brief  установка режима радобы ДеКоР
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  mode: режим работы
  */
void pn_dcr_set_mode(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode)
{
	switch(mode){
		case DCR_MODE_DEFAULT:
			pn_dcr_ptr->status &= (~0x000F);
			pn_dcr_ptr->status |= DCR_MODE_DEFAULT & 0x000F;
			memcpy((uint8_t*)&pn_dcr_ptr->fl_task.work, (uint8_t*)&pn_dcr_ptr->fl_task.default_flt,  sizeof(type_PNDCR_FlightTask));
			pn_dcr_ptr->fl_task.step_num = 0;
			pn_dcr_ptr->fl_task.pause_ms = 500;
			break;
		case DCR_MODE_FLIGHT_TASK:
			pn_dcr_ptr->status &= (~0x000F);
			pn_dcr_ptr->status |= DCR_MODE_FLIGHT_TASK & 0x000F;
			memcpy((uint8_t*)&pn_dcr_ptr->fl_task.work, (uint8_t*)&pn_dcr_ptr->fl_task.can,  sizeof(type_PNDCR_FlightTask));
			pn_dcr_ptr->fl_task.step_num = 0;
			pn_dcr_ptr->fl_task.pause_ms = 500;
			break;
		case DCR_MODE_PAUSE:
			pn_dcr_ptr->status &= (~0x000F);
			pn_dcr_ptr->status |= DCR_MODE_PAUSE & 0x000F;
			break;
		case DCR_MODE_OFF:
			pn_dcr_ptr->status &= (~0x000F);
			pn_dcr_ptr->status |= DCR_MODE_OFF & 0x000F;
			pn_dcr_pwr_off(pn_dcr_ptr, PN_DCR_PWR_ALL);
			pn_dcr_ptr->fl_task.step_num = 0;
			pn_dcr_ptr->fl_task.pause_ms = 0;
			break;
	}
	return;
}

/**
  * @brief  функция для загрузки полетного задания ДеКоР из вне
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  flight_task: указатель на массив из sizeof(type_PNDCR_FlightTask)-байт, в котором содержится полетное задание
  */
void pn_dcr_load_can_flight_task(type_PN_DCR_model* pn_dcr_ptr, uint8_t *flight_task)
{
	uint8_t i;
	memcpy((uint8_t*)&pn_dcr_ptr->fl_task.can, flight_task, sizeof(type_PNDCR_FlightTask));
	for (i=0; i<128; i++){
		pn_dcr_ptr->fl_task.can.step[i].pause_ms = __REV(pn_dcr_ptr->fl_task.can.step[i].pause_ms);
	}

}

/**
  * @brief  обработка состояний ДеКоР: состояния питания, наличие ошибок, полетного задания
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @param  time_step_ms: шаг в мс для обработки полиетного задания dcr
  */
void pn_dcr_process(type_PN_DCR_model* pn_dcr_ptr, uint32_t time_step_ms) 
{
	uint8_t error;
	//проверка питания
	if ((pn_dcr_ptr->pwr_check_timeout_ms > 0) && (pn_dcr_ptr->pwr_check_timeout_ms <= PN_DCR_PWR_TIMEOUT_MS)){
		pn_dcr_ptr->pwr_check_timeout_ms -= time_step_ms;
	}
	else{
		pn_dcr_ptr->pwr_check_timeout_ms = 0;
	}
	error = pn_dcr_pwr_check(pn_dcr_ptr, pn_dcr_ptr->pwr_check_timeout_ms);
	_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_PWR, error);
	// обработка полетного задания
	switch(pn_dcr_ptr->status & 0x000F){
		case DCR_MODE_DEFAULT:
		case DCR_MODE_FLIGHT_TASK:
			if (pn_dcr_ptr->fl_task.pause_ms >= time_step_ms){
				pn_dcr_ptr->fl_task.pause_ms -= time_step_ms;
			}
			else if(pn_dcr_ptr->fl_task.pause_ms > 0){
				pn_dcr_ptr->fl_task.pause_ms = 0;
			}
			else {
				if (pn_dcr_run_step_function(pn_dcr_ptr)){
				}
				else {
					_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_FL_TASK_ERROR, 0);
				}
			}
			break;
		case DCR_MODE_PAUSE:
			break;
		case DCR_MODE_OFF:
			break;
		default:
			_pn_dcr_error_collector(pn_dcr_ptr, PN_DCR_ERR_WRONG_MODE, 0);
			break;
	}
	//
}

/**
  * @brief  функция для загрузки полетного задания ДеКоР из вне
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
	* @retval статус выполнения функции: 1 - все выполненно, 0 - ошибка выполенения
  */
uint8_t pn_dcr_run_step_function(type_PN_DCR_model* pn_dcr_ptr)
{
	uint8_t type, cmd, data[8], len;
	uint32_t pause_ms = 0;
	type_PN_DCR_frame frame;
	uint8_t report = 0;
	//
	type = pn_dcr_ptr->fl_task.work.step[pn_dcr_ptr->fl_task.step_num].type;
	cmd = pn_dcr_ptr->fl_task.work.step[pn_dcr_ptr->fl_task.step_num].cmd;
	pause_ms = pn_dcr_ptr->fl_task.work.step[pn_dcr_ptr->fl_task.step_num].pause_ms;
	memcpy(data, pn_dcr_ptr->fl_task.work.step[pn_dcr_ptr->fl_task.step_num].data, 8);
	//
	pn_dcr_ptr->fl_task.pause_ms = pause_ms;
	if (((type == 0) && (cmd == 0)) || (pn_dcr_ptr->fl_task.step_num >= 128)){
		pn_dcr_ptr->fl_task.step_num = 0;
		report = 1;
	}
	else{
		pn_dcr_ptr->fl_task.step_num += 1;
		//
		switch (type){
			case DCR_PN_FLT_TYPE_PWR:
				if (cmd == 0x00){
					pn_dcr_pwr_off(pn_dcr_ptr, data[0]);
					report = 1;
				}
				else if (cmd == 0x01){
					pn_dcr_pwr_on(pn_dcr_ptr, data[0]);
					report = 1;
				}
				break;
			case DCR_PN_FLT_TYPE_UART:
				if (cmd == 0x01){
					len = _pn_dcr_form_frame(PN_DCR_UART_FRAME_SHORT, &frame, data[0], data[1], &data[2]);
					pn_dcr_uart_send(&pn_dcr_ptr->uart, (uint8_t*)&frame, len);
					report = 1;
				}
				break;
		}
	}
	//
	printf("DCR FlTask Step %d: type=%d, cmd=%d, pause_ms=%d\n", pn_dcr_ptr->fl_task.step_num, type, cmd, pause_ms);
	return report;
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
void  _pn_dcr_error_collector(type_PN_DCR_model* pn_dcr_ptr, uint16_t error, int16_t data)
{
  switch(error){
    case PN_DCR_ERR_NO_ERROR:
      pn_dcr_ptr->error_flags = PN_DCR_ERR_NO_ERROR;
      pn_dcr_ptr->error_cnt = 0;
      break;
    case PN_DCR_ERR_WRONG_FRAME_LENG:
			pn_dcr_ptr->error_flags |= PN_DCR_ERR_WRONG_FRAME_LENG;
      pn_dcr_ptr->error_cnt += 1;
      break;
		case PN_DCR_ERR_PWR:
			if (data){
				if (data != ((pn_dcr_ptr->error_flags >> 4) & 0xFF)){
					pn_dcr_ptr->error_cnt += 1;
				}
				pn_dcr_ptr->error_flags |= ((data&0xFF) << 4);
			}
      break;
		case PN_DCR_ERR_UART:
			if (data){
				if (data != ((pn_dcr_ptr->error_flags >> 12) & 0x0F)){
					pn_dcr_ptr->error_cnt += 1;
				}
				pn_dcr_ptr->error_flags |= ((data&0x0F) << 12);
			}
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