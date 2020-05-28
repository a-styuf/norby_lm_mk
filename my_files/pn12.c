/**
  ******************************************************************************
  * @file           : pn12.c
  * @version        : v1.0
  * @brief          : програмная модель для работы с ПН1.2 (Должна совпадаться с ПН1.1 процентов на 90%)
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn12.h"

/**
  * @brief  инийиализация полезной нагрузки 1.2
  * @param  pn12_ptr: указатель на структуру управления ПН1.2
  */
void pn_12_init(type_PN12_model* pn12_ptr, uint8_t num,  type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart)
{
	//инициализация дискретных сигналов на вход (ТМИ)
	pn12_ptr->input[0] = gpio_parameters_set(GPIOE, 8);
	pn12_ptr->input[1] = gpio_parameters_set(GPIOE, 9);
	pn12_ptr->input[2] = gpio_parameters_set(GPIOE, 10);
	pn12_ptr->input[3] = gpio_parameters_set(GPIOE, 11);
	//инициализация дискретных сигналов на выход (ИКУ)
	pn12_ptr->output[0] = gpio_parameters_set(GPIOG, 8);
	pn12_ptr->output[1] = gpio_parameters_set(GPIOG, 9);
	pn12_ptr->output[2] = gpio_parameters_set(GPIOG, 10);
	pn12_ptr->output[3] = gpio_parameters_set(GPIOG, 11);
	//
	pn_12_output_set(pn12_ptr, PN12_OUTPUT_DEFAULT);
	// установка канала управления питанием
	pn12_ptr->pwr_ch = pwr_ch_ptr;
	pwr_ch_set_bound(pwr_ch_ptr, PN_12_VOLT_MAX, PN_12_VOLT_MIN, PN_12_PWR_MAX, PN_12_PWR_MIN);
	// установка канала управления температурой
	pn12_ptr->tmp_ch = tmp_ch_ptr;
	tmp1075_set_bound(tmp_ch_ptr, PN_12_TEMP_HIGH, PN_12_TEMP_LOW, PN_12_TEMP_HYST);
		//
	pn12_ptr->status = 0;
	pn12_ptr->pwr_check_timeout_ms = 0;
	pn12_ptr->tmp_check_timeout_ms = 0;
	pn12_ptr->inhibit = 0;
	pn12_ptr->self_num = num;
	pn12_ptr->tmi_slice_number = 0;
	//
	_pn_12_error_collector(pn12_ptr, PN12_NO_ERROR, NULL);
	pn_12_report_reset(pn12_ptr);
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_reset_state(type_PN12_model* pn12_ptr)
{
	pn12_ptr->status = 0;
	pn12_ptr->pwr_check_timeout_ms = 0;
	pn12_ptr->tmp_check_timeout_ms = 0;
	pn12_ptr->tmi_slice_number = 0;
	//
	_pn_12_error_collector(pn12_ptr, PN12_NO_ERROR, NULL);
	pn_12_report_reset(pn12_ptr);
	pn_12_output_set(pn12_ptr, PN12_OUTPUT_DEFAULT);
	pn_12_pwr_off(pn12_ptr);
}

/**
  * @brief  поддержка работы програмной модели ПН
  * @param  pn12_ptr: указатель на структуру управления ПН
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_12_process(type_PN12_model* pn12_ptr, uint16_t period_ms)
{
	//проверка питания
	pn_12_pwr_process(pn12_ptr, period_ms);
	//проверка питания
	pn_12_tmp_process(pn12_ptr, period_ms);
	// поддержка протокола общения
}

/**
  * @brief  создание отчета о работе
  * @param  pn12_ptr: указатель на структуру управления ПН
  */
void pn_12_tmi_slice_create(type_PN12_model* pn12_ptr)
{
	//
	memset((uint8_t*)&pn12_ptr->tmi_slice, 0xFE, sizeof(type_PN12_TMI_slice));
	//
	pn12_ptr->tmi_slice.number  			= (pn12_ptr->tmi_slice_number++);
	pn12_ptr->tmi_slice.pl_type 			= pn12_ptr->self_num;
	pn12_ptr->tmi_slice.voltage 			= (pn12_ptr->pwr_ch->ina226.voltage >> 4) & 0xFF;
	pn12_ptr->tmi_slice.current 			= (pn12_ptr->pwr_ch->ina226.current >> 4) & 0xFF;
	pn12_ptr->tmi_slice.outputs 			= pn_12_get_outputs_state(pn12_ptr);
	pn12_ptr->tmi_slice.inputs 				= pn_12_get_inputs_state(pn12_ptr);
	pn12_ptr->tmi_slice.temp 					= (pn12_ptr->tmp_ch->temp >> 8) & 0xFF;
	pn12_ptr->tmi_slice.pl_error_cnt 	= (pn12_ptr->error_cnt >> 8) & 0xFF;
	pn12_ptr->tmi_slice.pl_errors 		= pn12_ptr->error_flags;
	pn12_ptr->tmi_slice.pl_status 		= pn12_ptr->status;
}

/**
  * @brief  получение среза телеметрии и проверка на необходимость отключения ПН
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
	* @param  slice: указатель на данные со срезом телеметрии
	* @retval 0 - нет необходимости отключать ПН, >0 - необходимо отключить ПН
  */
int8_t pn_12_tmi_slice_get_and_check(type_PN12_model* pn12_ptr, uint8_t *slice)
{
	int8_t ret_val = 0;
	// проверяем наличие ситуации при которой необходимо отключить циклограмму
	uint8_t tmp_error = pn_12_tmp_check(pn12_ptr);
	uint8_t pwr_rror = pn_12_pwr_check(pn12_ptr);
	if ((pn12_ptr->inhibit & PN_12_INH_TMP) == 0){
		if (tmp_error){
			ret_val += 1;
		}
	}
	if ((pn12_ptr->inhibit & PN_12_INH_PWR) == 0){
		if (pwr_rror){
			ret_val += 1;
		}
	}
	if (pn12_ptr->inhibit & PN_12_INH_SELF){
		ret_val += 1;
	}
	// создаем и копируем срез телеметрии
	pn_12_tmi_slice_create(pn12_ptr);
	memcpy(slice, (uint8_t*)&pn12_ptr->tmi_slice, sizeof(type_PN12_TMI_slice));
	//
	return ret_val;
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn12_ptr: указатель на структуру управления ПН
  */
void pn_12_report_create(type_PN12_model* pn12_ptr)
{
	//
	memset((uint8_t*)&pn12_ptr->report, 0xFE, sizeof(type_PN12_report));
	//
	pn12_ptr->report.status 			= pn12_ptr->status;
	pn12_ptr->report.error_flags 	= pn12_ptr->error_flags;
	pn12_ptr->report.err_cnt 			= pn12_ptr->error_cnt;
	pn12_ptr->report.inh 					= pn12_ptr->inhibit;
	pn12_ptr->report.voltage 			= pn12_ptr->pwr_ch->ina226.voltage;
	pn12_ptr->report.current 			= pn12_ptr->pwr_ch->ina226.current;
	pn12_ptr->report.temp 				= pn12_ptr->tmp_ch->temp;
	pn12_ptr->report.outputs 			= pn_12_get_outputs_state(pn12_ptr);
	pn12_ptr->report.inputs 			= pn_12_get_inputs_state(pn12_ptr);
	pn12_ptr->report.rsrv[0] 			= 0xFEFE;
	pn12_ptr->report.rsrv[1] 			= 0xFEFE;
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_report_reset(type_PN12_model* pn12_ptr)
{
	memset((uint8_t*)&pn12_ptr->report, 0x00, sizeof(type_PN12_report));
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
	* @param  inh: флаги отключения функционала ПН
  */
void pn_12_set_inh(type_PN12_model* pn12_ptr, uint8_t inh)
{
	pn12_ptr->inhibit = inh;
	//
	pn12_ptr->status &= ~(PN12_STATUS_INH);
	pn12_ptr->status |= ((inh<<4) & PN12_STATUS_INH);
	//
}

/**
  * @brief  возращает короткий статус, где  бит 0 - статус работы прибора, 1 - статус ошибок
  * @param  pn12_ptr: указатель на структуру управления ПН
	* @retval  короткий статус работы прибора
  */
uint8_t pn_12_get_short_status(type_PN12_model* pn12_ptr)
{
	uint8_t work_status, error_status;
	work_status = (pn12_ptr->status & PN12_STATUS_WORK) ? 1 : 0;
	error_status = (pn12_ptr->status & PN12_STATUS_ERROR) ? 1 : 0;
	return work_status | (error_status << 1);
}

/**
  * @brief  установка выходов управления ПН по битовой маске
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_output_set(type_PN12_model* pn12_ptr, uint8_t output_state)
{
	gpio_set(&pn12_ptr->output[0], (output_state >> 0) & 0x01);
	gpio_set(&pn12_ptr->output[1], (output_state >> 1) & 0x01);
	gpio_set(&pn12_ptr->output[2], (output_state >> 2) & 0x01);
	gpio_set(&pn12_ptr->output[3], (output_state >> 3) & 0x01);
}

/**
  * @brief  чтение состояния входов ПН
  * @param  pn12_ptr: указатель на структуру управления ПН1.2
  * @retval статус входов ПН: 0-C_TM_PWR_ERR, 1-C_TM_CPU_OK, 2-C_TM_INT, 3-C_TM_ERR
  */
uint8_t pn_12_get_inputs_state(type_PN12_model* pn12_ptr)
{
	uint8_t state = 0;
	state |= (gpio_get(&pn12_ptr->input[0]) & 0x01) << 0;
	state |= (gpio_get(&pn12_ptr->input[1]) & 0x01) << 1;
	state |= (gpio_get(&pn12_ptr->input[2]) & 0x01) << 2;
	state |= (gpio_get(&pn12_ptr->input[3]) & 0x01) << 3;
	return state;
}

/**
  * @brief  чтение состояния выходов ПН
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  * @retval статус выходов ПН: 0-C_TK_nRESET, 1-C_TK_SPI_SEL, 2-KU_2, 3-KU_3
  */
uint8_t pn_12_get_outputs_state(type_PN12_model* pn12_ptr)
{
	uint8_t state = 0;
	state |= (gpio_get(&pn12_ptr->output[0]) & 0x01) << 0;
	state |= (gpio_get(&pn12_ptr->output[1]) & 0x01) << 1;
	state |= (gpio_get(&pn12_ptr->output[2]) & 0x01) << 2;
	state |= (gpio_get(&pn12_ptr->output[3]) & 0x01) << 3;
	state |= (gpio_get(&pn12_ptr->pwr_ch->ena[0]) & 0x01) << 4;
	state |= (gpio_get(&pn12_ptr->pwr_ch->ena[1]) & 0x01) << 5;
	state |= (gpio_get(&pn12_ptr->pwr_ch->ena[2]) & 0x01) << 6;
	return state;
}

///*** функции поддержки проверок температуры ***///
/**
  * @brief  поддержка проверок температуры
  * @param  pn12_ptr: указатель на структуру управления ПН
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_12_tmp_process(type_PN12_model* pn12_ptr, uint16_t period_ms)
{
	if ((pn12_ptr->tmp_check_timeout_ms > 0) && (pn12_ptr->tmp_check_timeout_ms <= (0xFFFF - PN_12_TMP_PERIODICAL_TIMEOUT_MS))){
		pn12_ptr->tmp_check_timeout_ms -= period_ms;
	}
	else{
		pn12_ptr->tmp_check_timeout_ms = PN_12_TMP_PERIODICAL_TIMEOUT_MS; //нефиг долюбить эти ошибки постоянно, секунды вполне хватит
		pn_12_tmp_check(pn12_ptr);
	}
	
}

/**
  * @brief  проверка параметров температуры
  * @param  pn_12_ptr: указатель на структуру управления ПН
  * @retval ошибки каналов измерения температуры
  */
uint8_t pn_12_tmp_check(type_PN12_model* pn12_ptr)
{
	uint8_t report = 0;
	pn12_ptr->tmp_check_timeout_ms = PN_12_TMP_PERIODICAL_TIMEOUT_MS; //на случай асинхронного вызова
	if (tmp1075_get_error(pn12_ptr->tmp_ch, &report)){
		_pn_12_error_collector(pn12_ptr, PN12_TEMP_ERROR, report);
	}
	return report;
}

///*** функции поддержки работы с питанием ***///
/**
  * @brief  поддержка проверок питания
  * @param  pn12_ptr: указатель на структуру управления ПН
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_12_pwr_process(type_PN12_model* pn12_ptr, uint16_t period_ms)
{
	if ((pn12_ptr->pwr_check_timeout_ms > 0) && (pn12_ptr->pwr_check_timeout_ms <= (0xFFFF - PN_12_PWR_PERIODICAL_TIMEOUT_MS))){
		pn12_ptr->pwr_check_timeout_ms -= period_ms;
	}
	else{
		pn12_ptr->pwr_check_timeout_ms = PN_12_PWR_PERIODICAL_TIMEOUT_MS; //нефиг долюбить эти ошибки постоянно, секунды вполне хватит
		pn_12_pwr_check(pn12_ptr);
	}
}

/**
  * @brief  проверка параметров питания
  * @param  pn12_ptr: указатель на структуру управления ПН
  * @retval  ошибки каналов питания
  */
uint8_t pn_12_pwr_check(type_PN12_model* pn12_ptr)
{
	uint8_t report = 0;
	pn12_ptr->pwr_check_timeout_ms = PN_12_PWR_PERIODICAL_TIMEOUT_MS; //на случай асинхронного вызова
	if (pwr_ch_get_error(pn12_ptr->pwr_ch, &report)){
		_pn_12_error_collector(pn12_ptr, PN12_ERR_PWR, report);
	}
	return report;
}

/**
  * @brief  включение питания ПН
  * @param  pn12_ptr: указатель на структуру управления ПН
  */
void pn_12_pwr_on(type_PN12_model* pn12_ptr)
{
	pwr_ch_on_off(pn12_ptr->pwr_ch, 0x01);
	pn12_ptr->pwr_check_timeout_ms = PN_12_PWR_ON_OFF_TIMEOUT_MS;
	//
	pn12_ptr->status |= (PN12_STATUS_WORK);
	//
}

/**
  * @brief  отключение питания ПН
  * @param  pn12_ptr: указатель на структуру управления ПН
  */
void pn_12_pwr_off(type_PN12_model* pn12_ptr)
{
	pwr_ch_on_off(pn12_ptr->pwr_ch, 0x00);
	pn12_ptr->pwr_check_timeout_ms = PN_12_PWR_ON_OFF_TIMEOUT_MS;
	//
	pn12_ptr->status &= ~(PN12_STATUS_WORK);
	//
}

///*** Cfg ***///
/**
  * @brief  получение параметров работы прибора для сохранения в ПЗУ
  * @param  pn12_ptr: указатель на структуру управления ПН_ДКР
  * @param  cfg: укзаатель на структуру с параметрами ДеКоР
  * @retval  1 - ОК, 0 - ошибка
  */
uint8_t pn_12_get_cfg(type_PN12_model* pn12_ptr, uint8_t *cfg)
{
	memset((uint8_t*)&pn12_ptr->cfg, 0xFE, sizeof(type_PN12_сfg));
	pn12_ptr->cfg.inhibit = pn12_ptr->inhibit;
	//
	memcpy(cfg, (uint8_t*)&pn12_ptr->cfg, sizeof(type_PN12_сfg));
	//
	return 1;
}

/**
  * @brief  получение параметров работы прибора для сохранения в ПЗУ
  * @param  pn12_ptr: указатель на структуру управления ПН_ДКР
  * @param  cfg: укзаатель на структуру с параметрами ДеКоР
  * @retval  1 - ОК, 0 - ошибка (заготовка под проверку валидности данных)
  */
uint8_t pn_12_set_cfg(type_PN12_model* pn12_ptr, uint8_t *cfg)
{
	//
	memcpy((uint8_t*)&pn12_ptr->loaded_cfg, (uint8_t*)cfg, sizeof(type_PN12_сfg));
	pn_12_set_inh(pn12_ptr, pn12_ptr->loaded_cfg.inhibit);
	//
	return 1;
}

///*** функции внутреннего назаначения ***///
/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  app_lvl_ptr: указатель на структуру управления УРОВНЕМ ПРИЛОЖЕНИЯ
  * @param  error: ошибка, согласно define-ам PN_12_... в .h
  * @param  data: ошибки из других источников
  */
void  _pn_12_error_collector(type_PN12_model* pn12_ptr, uint16_t error, int16_t data)
{
  switch(error){
    case PN12_NO_ERROR:
      pn12_ptr->error_flags = PN12_NO_ERROR;
      pn12_ptr->error_cnt = 0;
      break;
		case PN12_STM1_ERROR:
		case PN12_STM2_ERROR:
		case PN12_STM3_ERROR:
		case PN12_STM4_ERROR:
			pn12_ptr->error_flags |= error;
      pn12_ptr->error_cnt += 1;
      break;
    case PN12_ERR_PWR:
			if (data){
				if (data != ((pn12_ptr->error_flags >> 4) & 0xF)){
					pn12_ptr->error_cnt += 1;
				}
				pn12_ptr->error_flags |= ((data&0xF) << 4);
			}
			break;
		case PN12_TEMP_ERROR:
			if (data){
				if (data != ((pn12_ptr->error_flags >> 8) & 0xF)){
					pn12_ptr->error_cnt += 1;
				}
				pn12_ptr->error_flags |= ((data&0xF) << 8);
			}
			break;
    case PN12_INTERFACE_ERROR:
    case PN12_APP_LVL_ERROR:
    case PN12_TR_LVL_ERROR:
			pn12_ptr->error_flags |= error;
      pn12_ptr->error_cnt += 1;
      break;
    default:
      pn12_ptr->error_flags |= PN12_OTHER_ERROR;
      pn12_ptr->error_cnt += 1;
      break;
  }
	//
	if (error != PN12_NO_ERROR){
		pn12_ptr->status &= ~PN12_STATUS_ERROR;
		pn12_ptr->status |= PN12_STATUS_ERROR;
	}
}

///*** Debug tests ***///
/**
  * @brief  функция для проверки переферии ПН12, !блокирующая! только для отладки
  * @note   проверка проводится при выходе информационного интерфейса с выхода на вход, при ИКУ подключенных к ТМ (по возможности)
	* 				оставшиеся сигналы ТМ подключаются по желанию
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_dbg_test(type_PN12_model* pn12_ptr)
{
	uint8_t test_data[16]={0xAA, 0x55, 0x02, 0x03}, receive_data[16]={0};
		pwr_ch_on_off_separatly(pn12_ptr->pwr_ch, 0x07);
	HAL_Delay(2000);
	///***  проверка gpio ***///
	HAL_Delay(500);
	pn_12_output_set(pn12_ptr, 0x0F);
	HAL_Delay(500);
	printf("Check gpio 0xF:");
	printf(": output 0x%02X, input 0x%02X\n", pn_12_get_outputs_state(pn12_ptr), pn_12_get_inputs_state(pn12_ptr));
	//
	HAL_Delay(500);
	pn_12_output_set(pn12_ptr, 0x00);
	HAL_Delay(500);
	printf("Check gpio 0x0:");
	printf(": output 0x%02X, input 0x%02X\n", pn_12_get_outputs_state(pn12_ptr), pn_12_get_inputs_state(pn12_ptr));
	///***  проверка uart ***///
	HAL_UART_Transmit_IT(pn12_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn12_ptr->interface.tr_lvl.huart, receive_data, 4, 200);
	printf("Test UART:\n");
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	HAL_Delay(2000);
}
