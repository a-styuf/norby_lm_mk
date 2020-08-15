/**
  ******************************************************************************
  * @file           : pn20.c
  * @version        : v1.0
  * @brief          : програмная модель для работы с ПН2.0 (Должна совпадаться с ПН1.1 процентов на 70% исключая протокол общения)
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn20.h"

/**
  * @brief  инийиализация полезной нагрузки ПН2.0
  * @param  pn20_ptr: указатель на структуру управления ПН2.0
  */
void pn_20_init(type_PN20_model* pn20_ptr, uint8_t num, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart)
{
		//инициализация дискретных сигналов на вход (ТМИ)
		pn20_ptr->input[0] = gpio_parameters_set(GPIOE, 12);
		pn20_ptr->input[1] = gpio_parameters_set(GPIOE, 13);
		pn20_ptr->input[2] = gpio_parameters_set(GPIOE, 14);
		pn20_ptr->input[3] = gpio_parameters_set(GPIOE, 15);
		//инициализация дискретных сигналов на выход (ИКУ)
		pn20_ptr->output[0] = gpio_parameters_set(GPIOG, 12);
		pn20_ptr->output[1] = gpio_parameters_set(GPIOG, 13);
		pn20_ptr->output[2] = gpio_parameters_set(GPIOG, 14);
		pn20_ptr->output[3] = gpio_parameters_set(GPIOG, 15);
	//
	pn_20_output_set(pn20_ptr, PN20_OUTPUT_DEFAULT);
	// установка канала управления питанием
	pn20_ptr->pwr_ch = pwr_ch_ptr;
	pwr_ch_set_bound(pwr_ch_ptr, PN_20_VOLT_MAX, PN_20_VOLT_MIN, PN_20_PWR_MAX, PN_20_PWR_MIN);
	// установка канала управления температурой
	pn20_ptr->tmp_ch = tmp_ch_ptr;
	tmp1075_set_bound(tmp_ch_ptr, PN_20_TEMP_HIGH, PN_20_TEMP_LOW, PN_20_TEMP_HYST);
		//
	pn20_ptr->status = 0;
	pn20_ptr->pwr_check_timeout_ms = 0;
	pn20_ptr->tmp_check_timeout_ms = 0;
	pn20_ptr->inhibit = 0;
	pn20_ptr->self_num = num;
	pn20_ptr->tmi_slice_number = 0;
	//
	_pn_20_error_collector(pn20_ptr, PN20_NO_ERROR, NULL);
	pn_20_report_reset(pn20_ptr);
	//
	pn_20_int_init(pn20_ptr, huart);
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn20_ptr: указатель на структуру управления ПН
  */
void pn_20_reset_state(type_PN20_model* pn20_ptr)
{
	pn20_ptr->status = 0;
	pn20_ptr->pwr_check_timeout_ms = 0;
	pn20_ptr->tmp_check_timeout_ms = 0;
	pn20_ptr->tmi_slice_number = 0;
	//
	_pn_20_error_collector(pn20_ptr, PN20_NO_ERROR, NULL);
	pn_20_report_reset(pn20_ptr);
	pn_20_output_set(pn20_ptr, PN20_OUTPUT_DEFAULT);
	pn_20_pwr_off(pn20_ptr);
	pn_20_set_inh(pn20_ptr, 0x00);
}

/**
  * @brief  поддержка работы програмной модели ПН
  * @param  pn20_ptr: указатель на структуру управления ПН
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_20_process(type_PN20_model* pn20_ptr, uint16_t period_ms)
{
	//проверка питания
	pn_20_pwr_process(pn20_ptr, period_ms);
	//проверка питания
	pn_20_tmp_process(pn20_ptr, period_ms);
	// поддержка протокола общения
	pn_20_interface_process(pn20_ptr, period_ms);
}

/**
  * @brief  создание отчета о работе
  * @param  pn20_ptr: указатель на структуру управления ПН
  */
void pn_20_tmi_slice_create(type_PN20_model* pn20_ptr)
{
	//
	memset((uint8_t*)&pn20_ptr->tmi_slice, 0xFE, sizeof(type_PN20_TMI_slice));
	//
	pn20_ptr->tmi_slice.number  			= (pn20_ptr->tmi_slice_number++);
	pn20_ptr->tmi_slice.pl_type 			= pn20_ptr->self_num;
	pn20_ptr->tmi_slice.voltage 			= (pn20_ptr->pwr_ch->ina226.voltage >> 4) & 0xFF;
	pn20_ptr->tmi_slice.current 			= (pn20_ptr->pwr_ch->ina226.current >> 4) & 0xFF;
	pn20_ptr->tmi_slice.outputs 			= pn_20_get_outputs_state(pn20_ptr);
	pn20_ptr->tmi_slice.inputs 				= pn_20_get_inputs_state(pn20_ptr);
	pn20_ptr->tmi_slice.temp 					= (pn20_ptr->tmp_ch->temp >> 8) & 0xFF;
	pn20_ptr->tmi_slice.pl_error_cnt 	= pn20_ptr->error_cnt;
	pn20_ptr->tmi_slice.pl_errors 		= pn20_ptr->error_flags;
	pn20_ptr->tmi_slice.pl_status 		= pn20_ptr->status;
}

/**
  * @brief  получение среза телеметрии и проверка на необходимость отключения ПН
  * @param  pn20_ptr: указатель на структуру управления ПН
	* @param  slice: указатель на данные со срезом телеметрии
	* @retval 0 - нет необходимости отключать ПН, >0 - необходимо отключить ПН
  */
int8_t pn_20_tmi_slice_get_and_check(type_PN20_model* pn20_ptr, uint8_t *slice)
{
	int8_t ret_val = 0;
	// проверяем наличие ситуации при которой необходимо отключить циклограмму
	uint8_t tmp_error = pn_20_tmp_check(pn20_ptr);
	uint8_t pwr_rror = pn_20_pwr_check(pn20_ptr);
	if ((pn20_ptr->inhibit & PN_20_INH_TMP) == 0){
		if (tmp_error){
			ret_val += 1;
		}
	}
	if ((pn20_ptr->inhibit & PN_20_INH_PWR) == 0){
		if (pwr_rror){
			ret_val += 1;
		}
	}
	if (pn20_ptr->inhibit & PN_20_INH_SELF){
		ret_val += 1;
	}
	// создаем и копируем срез телеметрии
	pn_20_tmi_slice_create(pn20_ptr);
	memcpy(slice, (uint8_t*)&pn20_ptr->tmi_slice, sizeof(type_PN20_TMI_slice));
	//
	return ret_val;
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_report_create(type_PN20_model* pn20_ptr)
{
	//
	memset((uint8_t*)&pn20_ptr->report, 0xFE, sizeof(type_PN20_report));
	//
	pn20_ptr->report.status 			= pn20_ptr->status;
	pn20_ptr->report.error_flags 	= pn20_ptr->error_flags;
	pn20_ptr->report.err_cnt 			= pn20_ptr->error_cnt;
	pn20_ptr->report.inh 					= pn20_ptr->inhibit;
	pn20_ptr->report.voltage 			= pn20_ptr->pwr_ch->ina226.voltage;
	pn20_ptr->report.current 			= pn20_ptr->pwr_ch->ina226.current;
	pn20_ptr->report.temp 				= pn20_ptr->tmp_ch->temp;
	pn20_ptr->report.outputs 			= pn_20_get_outputs_state(pn20_ptr);
	pn20_ptr->report.inputs 			= pn_20_get_inputs_state(pn20_ptr);
	pn20_ptr->report.rsrv[0] 			= 0xFEFE;
	pn20_ptr->report.rsrv[1] 			= 0xFEFE;
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn20_ptr: указатель на структуру управления ПН
  */
void pn_20_report_reset(type_PN20_model* pn20_ptr)
{
	memset((uint8_t*)&pn20_ptr->report, 0x00, sizeof(type_PN20_report));
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn20_ptr: указатель на структуру управления ПН
	* @param  inh: флаги отключения функционала ПН
  */
void pn_20_set_inh(type_PN20_model* pn20_ptr, uint8_t inh)
{
	pn20_ptr->inhibit = inh;
	//
	pn20_ptr->status &= ~(PN20_STATUS_INH);
	pn20_ptr->status |= ((inh<<4) & PN20_STATUS_INH);
	//
}

/**
  * @brief  возращает короткий статус, где  бит 0 - статус работы прибора, 1 - статус ошибок
  * @param  pn12_ptr: указатель на структуру управления ПН
	* @retval  короткий статус работы прибора
  */
uint8_t pn_20_get_short_status(type_PN20_model* pn20_ptr)
{
	uint8_t work_status, error_status;
	work_status = (pn20_ptr->status & PN20_STATUS_WORK) ? 1 : 0;
	error_status = (pn20_ptr->status & PN20_STATUS_ERROR) ? 1 : 0;
	return work_status | (error_status << 1);
}

/**
  * @brief  установка выходов управления ПН по битовой маске
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_output_set(type_PN20_model* pn20_ptr, uint8_t output_state)
{
	gpio_set(&pn20_ptr->output[0], (output_state >> 0) & 0x01);
	gpio_set(&pn20_ptr->output[1], (output_state >> 1) & 0x01);
	gpio_set(&pn20_ptr->output[2], (output_state >> 2) & 0x01);
	gpio_set(&pn20_ptr->output[3], (output_state >> 3) & 0x01);
}

/**
  * @brief  чтение состояния входов ПН
  * @param  pn20_ptr: указатель на структуру управления ПН1.2
  * @retval статус входов ПН: 0-C_TM_PWR_ERR, 1-C_TM_CPU_OK, 2-C_TM_INT, 3-C_TM_ERR
  */
uint8_t pn_20_get_inputs_state(type_PN20_model* pn20_ptr)
{
	uint8_t state = 0;
	state |= (gpio_get(&pn20_ptr->input[0]) & 0x01) << 0;
	state |= (gpio_get(&pn20_ptr->input[1]) & 0x01) << 1;
	state |= (gpio_get(&pn20_ptr->input[2]) & 0x01) << 2;
	state |= (gpio_get(&pn20_ptr->input[3]) & 0x01) << 3;
	return state;
}

/**
  * @brief  чтение состояния выходов ПН
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  * @retval статус выходов ПН: 0-C_TK_nRESET, 1-C_TK_SPI_SEL, 2-KU_2, 3-KU_3
  */
uint8_t pn_20_get_outputs_state(type_PN20_model* pn20_ptr)
{
	uint8_t state = 0;
	state |= (gpio_get(&pn20_ptr->output[0]) & 0x01) << 0;
	state |= (gpio_get(&pn20_ptr->output[1]) & 0x01) << 1;
	state |= (gpio_get(&pn20_ptr->output[2]) & 0x01) << 2;
	state |= (gpio_get(&pn20_ptr->output[3]) & 0x01) << 3;
	state |= (gpio_get(&pn20_ptr->pwr_ch->ena[0]) & 0x01) << 4;
	state |= (gpio_get(&pn20_ptr->pwr_ch->ena[1]) & 0x01) << 5;
	state |= (gpio_get(&pn20_ptr->pwr_ch->ena[2]) & 0x01) << 6;
	return state;
}


///*** функции поддержки проверок температуры ***///
/**
  * @brief  поддержка проверок температуры
  * @param  pn20_ptr: указатель на структуру управления ПН
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_20_tmp_process(type_PN20_model* pn20_ptr, uint16_t period_ms)
{
	if ((pn20_ptr->tmp_check_timeout_ms > 0) && (pn20_ptr->tmp_check_timeout_ms <= (0xFFFF - PN_20_TMP_PERIODICAL_TIMEOUT_MS))){
		pn20_ptr->tmp_check_timeout_ms -= period_ms;
	}
	else{
		pn20_ptr->tmp_check_timeout_ms = PN_20_TMP_PERIODICAL_TIMEOUT_MS; //нефиг долюбить эти ошибки постоянно, секунды вполне хватит
		pn_20_tmp_check(pn20_ptr);
	}
	
}

/**
  * @brief  проверка параметров температуры
  * @param  pn20_ptr: указатель на структуру управления ПН
  * @retval ошибки каналов измерения температуры
  */
uint8_t pn_20_tmp_check(type_PN20_model* pn20_ptr)
{
	uint8_t report = 0;
	pn20_ptr->tmp_check_timeout_ms = PN_20_TMP_PERIODICAL_TIMEOUT_MS; //на случай асинхронного вызова
	if (tmp1075_get_error(pn20_ptr->tmp_ch, &report)){
		_pn_20_error_collector(pn20_ptr, PN20_TEMP_ERROR, report);
	}
	return report;
}

///*** функции поддержки работы с питанием ***///
/**
  * @brief  поддержка проверок питания
  * @param  pn20_ptr: указатель на структуру управления ПН
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_20_pwr_process(type_PN20_model* pn20_ptr, uint16_t period_ms)
{
	if ((pn20_ptr->pwr_check_timeout_ms > 0) && (pn20_ptr->pwr_check_timeout_ms <= (0xFFFF - PN_20_PWR_PERIODICAL_TIMEOUT_MS))){
		pn20_ptr->pwr_check_timeout_ms -= period_ms;
	}
	else{
		pn20_ptr->pwr_check_timeout_ms = PN_20_PWR_PERIODICAL_TIMEOUT_MS; //нефиг долюбить эти ошибки постоянно, секунды вполне хватит
		pn_20_pwr_check(pn20_ptr);
	}
}

/**
  * @brief  проверка параметров питания
  * @param  pn20_ptr: указатель на структуру управления ПН
  * @retval  ошибки каналов питания
  */
uint8_t pn_20_pwr_check(type_PN20_model* pn20_ptr)
{
	uint8_t report = 0;
	pn20_ptr->pwr_check_timeout_ms = PN_20_PWR_PERIODICAL_TIMEOUT_MS; //на случай асинхронного вызова
	if (pwr_ch_get_error(pn20_ptr->pwr_ch, &report)){
		_pn_20_error_collector(pn20_ptr, PN20_ERR_PWR, report);
	}
	return report;
}

/**
  * @brief  включение питания ПН
  * @param  pn20_ptr: указатель на структуру управления ПН
  */
void pn_20_pwr_on(type_PN20_model* pn20_ptr)
{
	pwr_ch_on_off(pn20_ptr->pwr_ch, 0x01);
	pn20_ptr->pwr_check_timeout_ms = PN_20_PWR_ON_OFF_TIMEOUT_MS;
	//
	pn20_ptr->status |= (PN20_STATUS_WORK);
	//
}

/**
  * @brief  отключение питания ПН
  * @param  pn20_ptr: указатель на структуру управления ПН
  */
void pn_20_pwr_off(type_PN20_model* pn20_ptr)
{
	pwr_ch_on_off(pn20_ptr->pwr_ch, 0x00);
	pn20_ptr->pwr_check_timeout_ms = PN_20_PWR_ON_OFF_TIMEOUT_MS;
	//
	pn20_ptr->status &= ~(PN20_STATUS_WORK);
	//
}


///*** pl interface ***///

/**
  * @brief  инициализация интерфейса общения для ПН2.0
  * @param  pn_dcr_ptr: указатель на структуру управления ПН2.0
	* @param  uart_ptr: указатель на uart, используемый для данной ПН
  */
void pn_20_int_init(type_PN20_model* pn20_ptr, UART_HandleTypeDef *uart_ptr)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	// привязываем конкретный uart к полезной нагрузке
	int_ptr->huart = uart_ptr;
	// обнуляем переменные состояний
	int_ptr->tx_len = 0;
	int_ptr->rx_len = 0;
	int_ptr->row_rx_len = 0;
	int_ptr->rx_ptr = 0;
	int_ptr->rx_ptr_offset = 0;
	int_ptr->rx_timeout_flag = 0;
	int_ptr->rx_timeout = 0;
	int_ptr->rx_data_ready = 0;
	// заполнение очереди комманд для чтение телеметрии, результатов работы TAS и SD
	uint8_t sw_tmi_comm[PN_20_SW_TEST_U16_LENG] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
	memcpy((uint8_t*)int_ptr->sw_tmi_comm, (uint8_t*)sw_tmi_comm, PN_20_SW_TEST_U16_LENG);
	//
	uint8_t tas_start_comm[PN_20_TAS_START_U16_LENG] = {0x20};
	memcpy((uint8_t*)int_ptr->tas_start_comm, (uint8_t*)tas_start_comm, PN_20_TAS_START_U16_LENG);
	uint8_t tas_result_comm[PN_20_TAS_RESULT_U16_LENG] = {0x22, 0x23, 0x24, 0x25, 0x26};
	memcpy((uint8_t*)int_ptr->tas_result_comm, (uint8_t*)tas_result_comm, PN_20_TAS_RESULT_U16_LENG);
	//
	uint8_t sd_start_comm[PN_20_SD_START_U16_LENG] = {0x21};
	memcpy((uint8_t*)int_ptr->sd_start_comm, (uint8_t*)sd_start_comm, PN_20_SD_START_U16_LENG);
	uint8_t sd_result_comm[PN_20_SD_RESULT_U16_LENG] = {0x27, 0x28, 0x29, 0x2A, 0x2B};
	memcpy((uint8_t*)int_ptr->sd_result_comm, (uint8_t*)sd_result_comm, PN_20_SD_RESULT_U16_LENG);
	// чистим буферы приема и структуры кадров на всякий случай
	memset(int_ptr->tx_data, 0x00, sizeof(int_ptr->tx_data));
	memset(int_ptr->rx_data, 0x00, sizeof(int_ptr->rx_data));
	memset(&int_ptr->tx_msg.array, 0x00, sizeof(type_PN20_frame));
	memset(&int_ptr->rx_msg.array, 0x00, sizeof(type_PN20_frame));
}

/**
  * @brief  обработка и прием данных интерфейса ПН_20
  * @param  pn20_ptr указатель на структуру управления полезной нагрузкой
	* @param  period_ms задержка вызова функции
  */
void pn_20_interface_process(type_PN20_model* pn20_ptr, uint16_t period_ms)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	// обработка таймаута
	pn_20_int_rx_timeout_cb(pn20_ptr, period_ms);
	// анализ принятых данных
	if (int_ptr->rx_timeout != 0){
		//
		switch (pn_20_int_check_frame(pn20_ptr, int_ptr->rx_data+int_ptr->rx_ptr_offset, int_ptr->rx_ptr-int_ptr->rx_ptr_offset)){
			case PN_20_UART_RX_STATUS_SHORT:
				break;
			case PN_20_UART_RX_STATUS_OK:
				//
				int_ptr->rx_data_ready = 1;
				int_ptr->row_rx_len = int_ptr->rx_ptr;
				memcpy(int_ptr->row_rx_data, int_ptr->rx_data, int_ptr->row_rx_len);
				//
				int_ptr->rx_len = int_ptr->rx_ptr;
				pn_20_int_rx_ptr_reset(pn20_ptr);
				int_ptr->rx_cnt += 1;
				//
				int_ptr->rx_timeout = 0;
				int_ptr->rx_timeout_flag = 0;
				// складывание данных в структуру результата циклограммы
				if(int_ptr->comm_queue_flg){
					int_ptr->array_to_save[int_ptr->array_to_save_index] = (int_ptr->rx_msg.frame.data_hb << 8) + (int_ptr->rx_msg.frame.data_lb << 0);
				}		
				//
				break;
			case PN_20_UART_RX_STATUS_BAD_CRC:
				pn_20_int_rx_ptr_reset(pn20_ptr);
				_pn_20_error_collector(pn20_ptr, PN20_INT_ERR_BAD_FRAME, NULL);
				break;
			case PN_20_UART_RX_STATUS_BAD_EOT_SOF:
				int_ptr->rx_ptr_offset += 1;
				_pn_20_error_collector(pn20_ptr, PN20_INT_ERR_BAD_FRAME, NULL);
				break;
		}
	}
	else if((int_ptr->rx_timeout == 0) && (int_ptr->rx_timeout_flag == 1)){
		//
		int_ptr->rx_data_ready = 1;
		int_ptr->row_rx_len = int_ptr->rx_ptr;
		memcpy(int_ptr->row_rx_data, int_ptr->rx_data, int_ptr->row_rx_len);
		//
		pn_20_int_rx_ptr_reset(pn20_ptr);
		//
		int_ptr->rx_timeout = 0;
		int_ptr->rx_timeout_flag = 0;
		//
		_pn_20_error_collector(pn20_ptr, PN20_INT_ERR_TIMEOUT, NULL);
	}
	// запрос данных
	pn_20_comm_queue_process(pn20_ptr, period_ms);
}

/**
  * @brief  сбрс указателя rx_ptr c перезапуском чтения
  * @param  pn20_ptr: указатель на структуру управления ПН_20
  */
void pn_20_int_rx_ptr_reset(type_PN20_model* pn20_ptr)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	int_ptr->rx_ptr = 0;
	int_ptr->rx_ptr_offset = 0;
	HAL_UART_AbortReceive_IT(int_ptr->huart);
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_data + int_ptr->rx_ptr, 1);
}

/**
  * @brief  запись данных или запроса данных через uart
  * @param  pn20_ptr: указатель на структуру управления полезной нагрузкой
	* @param  insta_send_data: данные (128Б) из переменной CAN по следующему шаблону: 0-123 данные, 127 - ctrl_byte
  */
uint8_t pn_20_instasend(type_PN20_model* pn20_ptr, uint8_t* insta_send_data)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	//
	int_ptr->tx_len = insta_send_data[127];
	memcpy(int_ptr->tx_data, insta_send_data, int_ptr->tx_len);
	HAL_UART_Transmit_IT(int_ptr->huart, int_ptr->tx_data, int_ptr->tx_len);
	// устанавливаем таймаут на ответ
	int_ptr->rx_timeout = PN_20_UART_TIMEOUT_MS;
	int_ptr->rx_timeout_flag = 1;
	//
	return 0;
}

/**
  * @brief  отпрака команды по формату ПН2.0
  * @param  pn20_ptr: указатель на структуру управления ПН_20
	* @param  data_len: длина данных для отправки (1 или 2)
	* @param  data_hb: старший байт данных для отправки
	* @param  data_lb: младший байт данных для отправки
  */
void pn_20_int_send_frame(type_PN20_model* pn20_ptr, uint8_t data_len, uint8_t data_hb, uint8_t data_lb)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	if (int_ptr->rx_timeout == 0){
		//формируем кадр для отправки
		int_ptr->tx_msg.frame.sof 			= 0xAA;
		int_ptr->tx_msg.frame.len 			= data_len;
		int_ptr->tx_msg.frame.data_hb 	= data_hb;
		int_ptr->tx_msg.frame.data_lb 	= data_lb;
		int_ptr->tx_msg.frame.crc8 			= сrc8_calc_for_pn_20(int_ptr->tx_msg.array.frame, 4);
		int_ptr->tx_msg.frame.eot 			= 0x55;
		//отправляем данные
		memcpy(int_ptr->tx_data, &int_ptr->tx_msg, sizeof(type_PN20_frame));
		int_ptr->tx_len = sizeof(type_PN20_frame);
		HAL_UART_Transmit_IT(int_ptr->huart, int_ptr->tx_data, int_ptr->tx_len);
		// устанавливаем таймаут на ответ
		int_ptr->rx_timeout = PN_20_UART_TIMEOUT_MS;
		int_ptr->rx_timeout_flag = 1;
	}
	else{
		_pn_20_error_collector(pn20_ptr, PN20_INT_ERR_TIMEOUT, 0);
	}
}

/**
  * @brief  команда на старт тестирования TAS
  * @param  pn20_ptr: указатель на структуру управления ПН_20
	*/
void pn_20_send_start_tas_test(type_PN20_model* pn20_ptr)
{
	pn_20_int_send_frame(pn20_ptr, 1, 0x00, 0x20);
}

/**
  * @brief  команда на старт тестирования SD
  * @param  pn20_ptr: указатель на структуру управления ПН_20
	*/
void pn_20_send_start_sd_test(type_PN20_model* pn20_ptr)
{
	pn_20_int_send_frame(pn20_ptr, 1, 0x00, 0x21);
}

/**
  * @brief  запуск очереди отправки
  * @param  pn20_ptr указатель на структуру управления ПН_20
	* @param  comm_queue указатель на массив с очередью комманд
	* @param  comm_queue_leng дляина массива с очередю команд
	* @param  interval_ms временной шаг между командами на отправку (должен быть не меньше таймаута на ответ)
	*/
void pn_20_comm_queue_start(type_PN20_model* pn20_ptr, uint8_t* comm_queue, uint8_t comm_queue_leng, uint16_t* mem_to_save, uint16_t interval_ms)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	//
	memset(int_ptr->comm_queue_array, 0x00, 64);
	//
	int_ptr->comm_queue_leng = comm_queue_leng;
	int_ptr->queue_interval_ms = interval_ms;
	int_ptr->queue_step_time_ms = 0;
	int_ptr->comm_queue_cnt = 0;
	int_ptr->comm_queue_flg = 1;
	memcpy((uint8_t*)int_ptr->comm_queue_array, (uint8_t*)comm_queue, comm_queue_leng & 0x3F);
	int_ptr->array_to_save = mem_to_save;
}

/**
  * @brief  организация очереди отправки
  * @param  pn20_ptr указатель на структуру управления ПН_20
	* @param  pause_ms интервал вызова функции
	*/
void pn_20_comm_queue_process(type_PN20_model* pn20_ptr, uint16_t pause_ms)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	uint8_t queue_step_ready = 0;
	//
	if (int_ptr->comm_queue_flg){
		int_ptr->queue_step_time_ms += pause_ms;
		if (int_ptr->queue_step_time_ms >= int_ptr->queue_interval_ms)
		{
			int_ptr->queue_step_time_ms = 0; 
			queue_step_ready = 1;
		}
		//
		if (queue_step_ready){
			//
			int_ptr->array_to_save_index = int_ptr->comm_queue_cnt;
			if (int_ptr->comm_queue_cnt < int_ptr->comm_queue_leng){
				pn_20_int_send_frame(pn20_ptr, 1, 0x00, int_ptr->comm_queue_array[int_ptr->comm_queue_cnt]);
				int_ptr->array_to_save[int_ptr->array_to_save_index] = 0xFEFE;
			}
			else{
				//
			}
			if (int_ptr->comm_queue_cnt >= int_ptr->comm_queue_leng){
				int_ptr->comm_queue_cnt = 0;
				int_ptr->comm_queue_flg = 0;
				int_ptr->array_to_save = &int_ptr->array_to_save_trash;
			}
			int_ptr->comm_queue_cnt += 1;
			//
		}
		//
	}
}

/**
  * @brief  обработка колбэка на прием данных (для вызова в CB от прерывания на прием)
  * @param  int_ptr: указатель на структуру управления ПН_20
  */
void pn_20_int_rx_huart_cb(type_PN20_model* pn20_ptr)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	// инкрементация длины
	int_ptr->rx_ptr++;
	// запрос нового байта
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_data + int_ptr->rx_ptr, 1);
}

/**
  * @brief  обаработка таймаута на прием данных
  * @param  int_ptr: указатель на структуру управления ПН_20
	* @param  period_ms: значение периода вызова данной функции
  */
void pn_20_int_rx_timeout_cb(type_PN20_model* pn20_ptr, uint16_t period_ms)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	// обсчитываем изменение таймаута
	if (int_ptr->rx_timeout_flag == 1){
		if ((int_ptr->rx_timeout == 0) || (int_ptr->rx_timeout > (0xFFFFFFFF - period_ms))){
			int_ptr->rx_timeout = 0;
		}
		else{
			int_ptr->rx_timeout -= period_ms;
		}
	}
	else{
		//
	}
}

/**
  * @brief  проверка массива на наличие в ней пакета по протоколу ПН2.0
  * @param  pn20_ptr: указатель на структуру управления ПН_20
	* @param  data: указатель на массив данных для проверки
	* @param  len: длина массива данных для проверки
	* @retval  статус
  */
int8_t pn_20_int_check_frame(type_PN20_model* pn20_ptr, uint8_t* data, uint8_t len)
{
	type_PN20_frame in_frame;
	if (len < sizeof(type_PN20_frame)){ // если меньше 6-ти байт, то ждем еще данных
		return PN_20_UART_RX_STATUS_SHORT;
	}
	else if(len > 127){
		return PN_20_UART_RX_STATUS_BAD_EOT_SOF;
	}
	else{
		memcpy((uint8_t*)&in_frame, data, sizeof(type_PN20_frame));
		if ((in_frame.frame.sof == 0xAA) && (in_frame.frame.eot == 0x55)){
			if (сrc8_calc_for_pn_20(in_frame.array.frame, 5) == 0x00){
				memcpy((uint8_t*)&pn20_ptr->interface.rx_msg, (uint8_t*)&in_frame, sizeof(type_PN20_frame));
				return PN_20_UART_RX_STATUS_OK;
			}
			else{
				return PN_20_UART_RX_STATUS_BAD_CRC;
			}
		}
		else{
			return PN_20_UART_RX_STATUS_BAD_EOT_SOF;
		}
	}
}

/**
  * @brief  получение последних принятых данных для выкладывания на подадрес
  * @param  pn20_ptr: указатель на структуру управления ПН_20
	* @param  data: указатель на массив данных для проверки
	* @retval  длина принятых данных
  */
int8_t pn_20_int_get_last_data(type_PN20_model* pn20_ptr, uint8_t* data)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	//
	if(int_ptr->rx_data_ready){
		int_ptr->rx_data_ready = 0;
		memcpy(data, int_ptr->row_rx_data, int_ptr->row_rx_len);
		memset(int_ptr->row_rx_data, 0x00, 128);
		return int_ptr->row_rx_len;
	}
	return 0;
}

/**
  * @brief  обработка колбэка на успешную отправку данных данных
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_20
  */
void pn_20_int_tx_prcs_cb(type_PN20_model* pn20_ptr)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	int_ptr->tx_cnt++;
}

/**
  * @brief  обработка колбэка на ошибку при приеме или передачи
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_20
  */
void pn_20_int_err_prcs_cb(type_PN20_model* pn20_ptr)
{
	type_PN20_interface *int_ptr = &pn20_ptr->interface;
	uint32_t HAL_UART_ERROR;
	HAL_UART_ERROR = HAL_UART_GetError(int_ptr->huart);
	if (HAL_UART_ERROR & HAL_UART_ERROR_NE)
	{
		_pn_20_error_collector(pn20_ptr, PN20_INT_ERR_OTHER, 0);
		HAL_UART_ERROR &= ~HAL_UART_ERROR_NE;
	}
	else if (HAL_UART_ERROR){
		_pn_20_error_collector(pn20_ptr, PN20_INT_ERR_OTHER, 0);
	}
}

///*** Cfg ***///
/**
  * @brief  получение параметров работы прибора для сохранения в ПЗУ
  * @param  pn20_ptr: указатель на структуру управления ПН_20
  * @param  cfg: указатель на область памяти с конфигурацией
  * @retval  1 - ОК, 0 - ошибка
  */
uint8_t pn_20_get_cfg(type_PN20_model* pn20_ptr, uint8_t *cfg)
{
	memset((uint8_t*)&pn20_ptr->cfg, 0xFE, sizeof(type_PN20_сfg));
	pn20_ptr->cfg.inhibit = pn20_ptr->inhibit;
	//
	memcpy(cfg, (uint8_t*)&pn20_ptr->cfg, sizeof(type_PN20_сfg));
	//
	return 1;
}

/**
  * @brief  получение параметров работы прибора для сохранения в ПЗУ
  * @param  pn20_ptr: указатель на структуру управления ПН_20
  * @param  cfg: указатель на область памяти с конфигурацией
  * @retval  1 - ОК, 0 - ошибка (заготовка под проверку валидности данных)
  */
uint8_t pn_20_set_cfg(type_PN20_model* pn20_ptr, uint8_t *cfg)
{
	//
	memcpy((uint8_t*)&pn20_ptr->loaded_cfg, (uint8_t*)cfg, sizeof(type_PN20_сfg));
	pn_20_set_inh(pn20_ptr, pn20_ptr->loaded_cfg.inhibit);
	//
	return 1;
}

///*** функции внутреннего назаначения ***///
/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  app_lvl_ptr: указатель на структуру управления УРОВНЕМ ПРИЛОЖЕНИЯ
  * @param  error: ошибка, согласно define-ам PN_20_... в .h
  * @param  data: ошибки из других источников
  */
void _pn_20_error_collector(type_PN20_model* pn20_ptr, uint16_t error, int16_t data)
{
  switch(error){
    case PN20_NO_ERROR:
      pn20_ptr->error_flags = PN20_NO_ERROR;
      pn20_ptr->error_cnt = 0;
      break;
		case PN20_STM1_ERROR:
		case PN20_STM2_ERROR:
		case PN20_STM3_ERROR:
		case PN20_STM4_ERROR:
			pn20_ptr->error_flags |= error;
      pn20_ptr->error_cnt += 1;
      break;
    case PN20_ERR_PWR:
			if (data){
				if (data != ((pn20_ptr->error_flags >> 4) & 0xF)){
					pn20_ptr->error_cnt += 1;
				}
				pn20_ptr->error_flags |= ((data&0xF) << 4);
			}
			break;
		case PN20_TEMP_ERROR:
			if (data){
				if (data != ((pn20_ptr->error_flags >> 8) & 0xF)){
					pn20_ptr->error_cnt += 1;
				}
				pn20_ptr->error_flags |= ((data&0xF) << 8);
			}
			break;
    case PN20_INT_ERR_BAD_FRAME:
    case PN20_INT_ERR_TIMEOUT:
    case PN20_INT_ERR_OTHER:
			pn20_ptr->error_flags |= error;
      pn20_ptr->error_cnt += 1;
      break;
    default:
      pn20_ptr->error_flags |= PN20_OTHER_ERROR;
      pn20_ptr->error_cnt += 1;
      break;
  }
	//
	if (error != PN20_NO_ERROR){
		pn20_ptr->status &= ~PN20_STATUS_ERROR;
		pn20_ptr->status |= PN20_STATUS_ERROR;
	}
}

///*** debug test ***///
/**
  * @brief  функция для проверки переферии ПН20, !блокирующая! только для отладки
  * @note   проверка проводится при выходе информационного интерфейса с выхода на вход, при ИКУ подключенных к ТМ (по возможности)
	* 				оставшиеся сигналы ТМ подключаются по желанию
  * @param  pn20_ptr: указатель на структуру управления ПН
  */
void pn_20_dbg_test(type_PN20_model* pn20_ptr)
{
	uint8_t test_data[16]={0xAA, 0x55, 0x02, 0x03}, receive_data[16]={0};
	pwr_ch_on_off_separatly(pn20_ptr->pwr_ch, 0x07);
	HAL_Delay(2000);
	///***  проверка gpio ***///
	HAL_Delay(500);
	pn_20_output_set(pn20_ptr, 0x0F);
	HAL_Delay(500);
	printf("Check gpio 0xF: ");
	printf("output 0x%02X, input 0x%02X\n", pn_20_get_outputs_state(pn20_ptr), pn_20_get_inputs_state(pn20_ptr));
	//
	HAL_Delay(500);
	pn_20_output_set(pn20_ptr, 0x00);
	HAL_Delay(500);
	printf("Check gpio 0x0: ");
	printf("output 0x%02X, input 0x%02X\n", pn_20_get_outputs_state(pn20_ptr), pn_20_get_inputs_state(pn20_ptr));
	///***  проверка uart ***///
	HAL_UART_Transmit_IT(pn20_ptr->interface.huart, test_data, 4);
	HAL_UART_Receive(pn20_ptr->interface.huart, receive_data, 4, 200);

	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	pwr_ch_on_off_separatly(pn20_ptr->pwr_ch, 0x00);
	HAL_Delay(2000);
}
