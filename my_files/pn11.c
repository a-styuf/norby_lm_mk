/**
  ******************************************************************************
  * @file           : pn11.c
  * @version        : v1.0
  * @brief          : програмная модель для работы с ПН1.1А и ПН1.1Б ! без контроля питания !
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn11.h"
#include <stdio.h>

/**
  * @brief  инийиализация полезной нагрузки 1.1
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  * @param  num: номер ПН1.1: 0-А, 1-B
  */
void pn_11_init(type_PN11_model* pn11_ptr, uint8_t num, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart)
{
	//
	if (num == 1){ //Инициализациия ПН1.1_А
		//инициализация дискретных сигналов на вход (ТМИ)
		pn11_ptr->input[0] = gpio_parameters_set(GPIOE, 0);
		pn11_ptr->input[1] = gpio_parameters_set(GPIOE, 1);
		pn11_ptr->input[2] = gpio_parameters_set(GPIOE, 2);
		pn11_ptr->input[3] = gpio_parameters_set(GPIOE, 3);
		//инициализация дискретных сигналов на выход (ИКУ)
		pn11_ptr->output[0] = gpio_parameters_set(GPIOG, 0);
		pn11_ptr->output[1] = gpio_parameters_set(GPIOG, 1);
		pn11_ptr->output[2] = gpio_parameters_set(GPIOG, 2);
		pn11_ptr->output[3] = gpio_parameters_set(GPIOG, 3);
	}
	else if(num == 2){
		//инициализация дискретных сигналов на вход (ТМИ)
		pn11_ptr->input[0] = gpio_parameters_set(GPIOE, 4);
		pn11_ptr->input[1] = gpio_parameters_set(GPIOE, 5);
		pn11_ptr->input[2] = gpio_parameters_set(GPIOE, 6);
		pn11_ptr->input[3] = gpio_parameters_set(GPIOE, 7);
		//инициализация дискретных сигналов на выход (ИКУ)
		pn11_ptr->output[0] = gpio_parameters_set(GPIOG, 4);
		pn11_ptr->output[1] = gpio_parameters_set(GPIOG, 5);
		pn11_ptr->output[2] = gpio_parameters_set(GPIOG, 6);
		pn11_ptr->output[3] = gpio_parameters_set(GPIOG, 7);
	}
	pn_11_output_set(pn11_ptr, PN11_OUTPUT_DEFAULT);
	// установка канала управления питанием
	pn11_ptr->pwr_ch = pwr_ch_ptr;
	pwr_ch_set_bound(pwr_ch_ptr, PN_11_VOLT_MAX, PN_11_VOLT_MIN, PN_11_PWR_MAX, PN_11_PWR_MIN);
	// установка канала управления температурой
	pn11_ptr->tmp_ch = tmp_ch_ptr;
	//
	pn11_ptr->status = 0;
	_pn11_error_collector(pn11_ptr, PN11_NO_ERROR);
	pn_11_report_reset(pn11_ptr);
	// инициализация интерфейса общения
	pn_11_interface_init(pn11_ptr, huart);
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_reset_state(type_PN11_model* pn11_ptr)
{
	// сбрасываем переменные состояний
	pn11_ptr->status = 0;
	pn11_ptr->interrupt_timeout = 0;
	_pn11_error_collector(pn11_ptr, PN11_NO_ERROR);
	pn_11_report_reset(pn11_ptr);
	pn_11_interface_reset(pn11_ptr);
	pn_11_output_set(pn11_ptr, PN11_OUTPUT_DEFAULT);
	//
}

/**
  * @brief  поддержка работы програмной модели ПН
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_11_process(type_PN11_model* pn11_ptr, uint16_t period_ms)
{
	pn_11_interface_process(pn11_ptr, period_ms);
}

/**
  * @brief  установка выходов управления ПН по битовой маске
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_output_set(type_PN11_model* pn11_ptr, uint8_t output_state)
{
	gpio_set(&pn11_ptr->output[0], (output_state >> 0) & 0x01);
	gpio_set(&pn11_ptr->output[1], (output_state >> 1) & 0x01);
	gpio_set(&pn11_ptr->output[2], (output_state >> 2) & 0x01);
	gpio_set(&pn11_ptr->output[3], (output_state >> 3) & 0x01);
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_report_create(type_PN11_model* pn11_ptr)
{
	//
	memset((uint8_t*)&pn11_ptr->report, 0xFE, sizeof(type_PN11_report));
	//
	pn11_ptr->report.status 			= pn11_ptr->status;
	pn11_ptr->report.error_flags 	= pn11_ptr->error_flags;
	pn11_ptr->report.err_cnt 			= pn11_ptr->error_cnt;
	pn11_ptr->report.gap 					= 0xFE;
	pn11_ptr->report.voltage 			= pn11_ptr->pwr_ch->ina226.voltage;
	pn11_ptr->report.current 			= pn11_ptr->pwr_ch->ina226.current;
	pn11_ptr->report.temp 				= pn11_ptr->tmp_ch->temp;
	pn11_ptr->report.outputs 			= pn_11_get_outputs_state(pn11_ptr);
	pn11_ptr->report.inputs 			= pn_11_get_inputs_state(pn11_ptr);
	pn11_ptr->report.rsrv[0] 			= 0xFEFE;
	pn11_ptr->report.rsrv[1] 			= 0xFEFE;
}

/**
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_report_reset(type_PN11_model* pn11_ptr)
{
	memset((uint8_t*)&pn11_ptr->report, 0x00, sizeof(type_PN11_report));
}

/**
  * @brief  чтение состояния входов ПН
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  * @retval статус входов ПН: 0-INT, 1-PWR_ERROR, 2-WATCHDOG, 3-CPU_ERROR
  */
uint8_t pn_11_get_inputs_state(type_PN11_model* pn11_ptr)
{
	uint8_t state = 0;
	state |= (gpio_get(&pn11_ptr->input[0]) & 0x01) << 0;
	state |= (gpio_get(&pn11_ptr->input[1]) & 0x01) << 1;
	state |= (gpio_get(&pn11_ptr->input[2]) & 0x01) << 2;
	state |= (gpio_get(&pn11_ptr->input[3]) & 0x01) << 3;
	return state;
}

/**
  * @brief  чтение состояния выходов ПН
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  * @retval статус выходов ПН: 0-INT, 1-PWR_ERROR, 2-WATCHDOG, 3-CPU_ERROR
  */
uint8_t pn_11_get_outputs_state(type_PN11_model* pn11_ptr)
{
	uint8_t state = 0;
	state |= (gpio_get(&pn11_ptr->output[0]) & 0x01) << 0;
	state |= (gpio_get(&pn11_ptr->output[1]) & 0x01) << 1;
	state |= (gpio_get(&pn11_ptr->output[2]) & 0x01) << 2;
	state |= (gpio_get(&pn11_ptr->output[3]) & 0x01) << 3;
	state |= (gpio_get(&pn11_ptr->pwr_ch->ena[0]) & 0x01) << 4;
	state |= (gpio_get(&pn11_ptr->pwr_ch->ena[1]) & 0x01) << 5;
	state |= (gpio_get(&pn11_ptr->pwr_ch->ena[2]) & 0x01) << 6;
	return state;
}

/**
  * @brief  включение питания ПН1.1
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_pwr_on(type_PN11_model* pn11_ptr)
{
	pwr_ch_on_off(pn11_ptr->pwr_ch, 0x01);
}

/**
  * @brief  отключение питания ПН1.1
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_pwr_off(type_PN11_model* pn11_ptr)
{
	pwr_ch_on_off(pn11_ptr->pwr_ch, 0x00);
}

void pn_11_get_pwr_val(type_PN11_model* pn11_ptr)
{

}

void pn_11_get_interrupt(type_PN11_model* pn11_ptr)
{

}

///*** функции поддержки интерфейса ***///
/**
  * @brief  инициализация интерфейса общения с ПН
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
  */
void pn_11_interface_init(type_PN11_model* pn11_ptr, UART_HandleTypeDef* huart)
{
	// инициализация уровня приложения и ниже
	app_lvl_init(&pn11_ptr->interface, huart);
	// инициализация параметров для последовательного чтения
	pn11_ptr->rd_seq_start_addr = 0;
	pn11_ptr->rd_seq_stop_addr = 0;
	pn11_ptr->rd_seq_leng = 0;
	pn11_ptr->rd_seq_mode = 0;
	pn11_ptr->rd_seq_curr_addr = 0;
	pn11_ptr->rd_seq_part_leng = 0;
	// зачищаем память для хранения памяти ПН
	memset((uint8_t*)&pn11_ptr->mem, 0x00, sizeof(type_PN_11_MEM));
}

/**
  * @brief  старт последовательного вычитывания данных из памяти ПН
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
  */
void pn_11_interface_reset(type_PN11_model* pn11_ptr)
{
	// инициализация уровня приложения и ниже
	app_lvl_reset(&pn11_ptr->interface);
	// инициализация параметров для последовательного чтения
	pn11_ptr->rd_seq_start_addr = 0;
	pn11_ptr->rd_seq_stop_addr = 0;
	pn11_ptr->rd_seq_leng = 0;
	pn11_ptr->rd_seq_mode = 0;
	pn11_ptr->rd_seq_curr_addr = 0;
	pn11_ptr->rd_seq_part_leng = 0;
	// зачищаем память для хранения памяти ПН
	memset((uint8_t*)&pn11_ptr->mem, 0x00, sizeof(type_PN_11_MEM));
}

/**
  * @brief  синхронизация транспортного протокола
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_interface_synch(type_PN11_model* pn11_ptr)
{
	tr_lvl_synch(&pn11_ptr->interface.tr_lvl);
}

/**
  * @brief  поддержка работы интерфейса к ПН
  * @param  pn11_ptr: указатель на структуру управления транспортным уровнем
  * @param  period_ms: период, с которым вызавается данный обработчик
  */
void pn_11_interface_process(type_PN11_model* pn11_ptr, uint16_t period_ms)
{
	uint8_t leng;
	type_APP_LVL_PCT rx_frame;
	uint32_t mem_addr, mem_leng;
	//поддрежка работы транспортного уровня
  tr_lvl_process(&pn11_ptr->interface.tr_lvl, period_ms);
  //поддрежка работы уровня приложения
  app_lvl_process(&pn11_ptr->interface, period_ms);
  //поддержка работы с памятью ПН
	if(pn11_ptr->rd_seq_mode == 0){
			//
	}
	else if(pn11_ptr->rd_seq_mode == 1){
		//поддержка периодичности провреки состояния чтения памяти ПН
		if ((pn11_ptr->rd_seq_timeout > PN_11_READ_MEM_TIMEOUT_MS)){
			// читаем старый запрос
			leng = app_lvl_get_rx_frame(&pn11_ptr->interface, &rx_frame);
			if (leng){
				// printf("addr %08X, ctrl %08X, data %08X\n", rx_frame.addr, rx_frame.ctrl_byte, rx_frame.data[0]);
				mem_addr = (rx_frame.addr - APP_LVL_ADDR_OFFSET) / 4; // на уровне приложения адрессация побайтова, а нам нужна по u32 (4-байта)
				mem_leng = (rx_frame.ctrl_byte & 0x3F) + 1;
				// проверяем корректность адреса и длины
				if (((mem_addr + mem_leng)*4) < sizeof(type_PN_11_MEM)){
					memcpy((uint8_t*)&pn11_ptr->mem.array.data[mem_addr], (uint8_t*)&rx_frame.data[0], mem_leng*4);
				}
				else{ // вылазим за допустиму робласть данных
					_pn11_error_collector(pn11_ptr, PN11_INTERFACE_ERROR);
				}
				pn11_ptr->rd_seq_mode = 0;
			}
			else{
				_pn11_error_collector(pn11_ptr, PN11_INTERFACE_ERROR);
			}
			// делаем новый запрос на чтение данных
			_pn_11_seq_read_request(pn11_ptr);
			// перезапускаем таймаут
			pn11_ptr->rd_seq_timeout = 0;
		}
		else{
			pn11_ptr->rd_seq_timeout += period_ms;
		}
	}
}

/**
  * @brief  старт последовательного вычитывания данных из памяти ПН
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
	* @param  start_addr: адрес для записи данных (байтовый адрес)
	* @param  u32_leng: адрес для записи данных
  */
void pn_11_seq_read_start(type_PN11_model* pn11_ptr, uint32_t start_addr, uint32_t u32_leng)
{
	//записываем общие настройки чтения
	pn11_ptr->rd_seq_start_addr = start_addr/4;
	pn11_ptr->rd_seq_stop_addr = start_addr/4 + u32_leng;
	pn11_ptr->rd_seq_leng = u32_leng;
	pn11_ptr->rd_seq_mode = 1;
	pn11_ptr->rd_seq_timeout = 0;
	//чистим память под запрос
	memset((uint8_t*)&pn11_ptr->mem.array.data[(start_addr - APP_LVL_ADDR_OFFSET)/4], 0x00, u32_leng*4);
	//делаем первый запрос в серии
	pn11_ptr->rd_seq_curr_addr = pn11_ptr->rd_seq_start_addr;
  _pn_11_seq_read_request(pn11_ptr);
}

/**
  * @brief  получение последнего пакета принятого интерфейсом полезной
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
  * @retval >0 длина последнего принятого пакета, 0 - пакет уже прочитан, или нулевой длины
  */
uint8_t pn_11_get_last_frame(type_PN11_model* pn11_ptr, uint8_t *data)
{
	return app_lvl_get_last_rx_frame(&pn11_ptr->interface, data);
}

/**
  * @brief  получение последнего пакета принятого интерфейсом полезной нагрузки, для выставления на подадрес CAN
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
  * @retval >0 длина последнего принятого пакета, 0 - пакет уже прочитан, или нулевой длины
  */
uint8_t pn_11_get_last_frame_in_128B_format(type_PN11_model* pn11_ptr, uint8_t *data)
{
	uint8_t leng = 0, u8_data[128]={0};
	uint32_t u32_data[32]  = {0};
	leng = app_lvl_get_last_rx_frame(&pn11_ptr->interface, u8_data);
	if (leng){
		for (uint8_t i=0; i<128/4; i++){
			u32_data[i] = __REV(*(uint32_t*)&u8_data[4*i]);
		}
		memcpy(data, (uint8_t*)u32_data, 128);
	}
	return leng;
}

/**
  * @brief  отправка запроса на чтение данных через app_lvl
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
	* @param  addr: адрес для записи данных
	* @param  u32_len: длинна данных для записи в uint32_t словах
  */
void pn_11_read_req_u32_data(type_PN11_model* pn11_ptr, uint32_t addr, uint8_t u32_len)
{
	app_lvl_read_req(&pn11_ptr->interface, addr, u32_len);
}

/**
  * @brief  запись данных через app_lvl
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
	* @param  addr: адрес для записи данных
	* @param  data: указатель на структуру управления полезной нагрузкой
	* @param  len: указатель на структуру управления полезной нагрузкой
  */
void pn_11_write_u32_data(type_PN11_model* pn11_ptr, uint32_t addr, uint32_t *u32_data, uint8_t u32_len)
{
	app_lvl_write(&pn11_ptr->interface, addr, u32_data, u32_len);
}

/**
  * @brief  запись данных или запроса данных через app_lvl
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
	* @param  insta_send_data: данные (128Б) из переменной СФТ по следующему шаблону: 0-3 - адрес, 4-123 данные, 127 - ctrl_byte
  */
uint8_t pn_11_can_instasend(type_PN11_model* pn11_ptr, uint8_t* insta_send_data)
{
	uint8_t u32_len, mode;
	uint32_t u32_data[APP_LVL_MAX_U32_DATA], addr;
	//
	u32_len = (insta_send_data[127] & 0x3F) + 1;
	if (u32_len > APP_LVL_MAX_U32_DATA) u32_len = APP_LVL_MAX_U32_DATA;
	for (uint8_t i=0; i < u32_len; i++){
		u32_data[i] = __REV(*(uint32_t*)&insta_send_data[4+4*i]);  // 4 - сдвиг из-за адреса, 4*i - сдвиг указателя по 4 байта
	}
	//
	addr = __REV(*(uint32_t*)&insta_send_data[0]);
	//
	mode = insta_send_data[127] >> 6;
	//
	switch(mode){
		case APP_LVL_MODE_READ:
			pn_11_read_req_u32_data(pn11_ptr, addr, u32_len);
			insta_send_data[124] = 0x01;
		break;
		case APP_LVL_MODE_WRITE:
			pn_11_write_u32_data(pn11_ptr, addr, u32_data, u32_len);
			insta_send_data[124] = 0x01;
		break;
	}
	return 0;
}

/**
  * @brief  старт последовательного вычитывания данных из памяти ПН
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
  */
void _pn_11_seq_read_request(type_PN11_model* pn11_ptr)
{
		if ((pn11_ptr->rd_seq_stop_addr - pn11_ptr->rd_seq_curr_addr) > APP_LVL_MAX_U32_DATA){
			pn11_ptr->rd_seq_part_leng = APP_LVL_MAX_U32_DATA;
		}
		else{
			pn11_ptr->rd_seq_part_leng = (pn11_ptr->rd_seq_stop_addr - pn11_ptr->rd_seq_curr_addr);
		}
		if ((pn11_ptr->rd_seq_part_leng > 0) && (pn11_ptr->rd_seq_curr_addr < pn11_ptr->rd_seq_stop_addr)){
			pn_11_read_req_u32_data(pn11_ptr, (pn11_ptr->rd_seq_curr_addr << 2), pn11_ptr->rd_seq_part_leng);
			pn11_ptr->rd_seq_curr_addr += pn11_ptr->rd_seq_part_leng;
		}
		else{
			pn11_ptr->rd_seq_mode = 0;
		}
}


///*** функции внутреннего назаначения ***///
/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  app_lvl_ptr: указатель на структуру управления УРОВНЕМ ПРИЛОЖЕНИЯ
  * @param  error: ошибка, согласно define-ам APP_LVL_... в .h
  */
void  _pn11_error_collector(type_PN11_model* pn11_ptr, uint16_t error)
{
  switch(error){
    case PN11_NO_ERROR:
      pn11_ptr->error_flags = APP_LVL_NO_ERR;
      pn11_ptr->error_cnt = 0;
      break;
    case PN11_TEMP_ERROR:
    case PN11_CURRENT_ERROR:
    case PN11_VOLTAGE_ERROR:
    case PN11_INTERFACE_ERROR:
    case PN11_INT_ERROR:
    case PN11_CPU_ERROR:
    case PN11_FPGA_ERROR:
			pn11_ptr->error_flags |= error;
      pn11_ptr->error_cnt += 1;
      break;
    default:
      pn11_ptr->error_flags |= PN11_OTHER_ERROR;
      pn11_ptr->error_cnt += 1;
      break;
  }
}

///*** Debug tests ***///
/**
  * @brief  функция для проверки переферии ПН11, !блокирующая! только для отладки
  * @note   проверка проводится при выходе информационного интерфейса с выхода на вход, при ИКУ подключенных к ТМ (по возможности)
	* 				оставшиеся сигналы ТМ подключаются по желанию
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_dbg_test(type_PN11_model* pn11_ptr)
{
	uint8_t test_data[16]={0xAA, 0x55, 0x02, 0x03}, receive_data[16]={0};
		pwr_ch_on_off_separatly(pn11_ptr->pwr_ch, 0x07);
	HAL_Delay(2000);
	///***  проверка gpio ***///
	HAL_Delay(500);
	pn_11_output_set(pn11_ptr, 0x0F);
	HAL_Delay(500);
	printf("Check gpio 0xF:");
	printf(" output 0x%02X, input 0x%02X\n", pn_11_get_outputs_state(pn11_ptr), pn_11_get_inputs_state(pn11_ptr));
	//
	HAL_Delay(500);
	pn_11_output_set(pn11_ptr, 0x00);
	HAL_Delay(500);
	printf("Check gpio 0x0:");
	printf(" output 0x%02X, input 0x%02X\n", pn_11_get_outputs_state(pn11_ptr), pn_11_get_inputs_state(pn11_ptr));
	///***  проверка uart ***///
	HAL_UART_Transmit_IT(pn11_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn11_ptr->interface.tr_lvl.huart, receive_data, 4, 200);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	HAL_Delay(2000);
}

/**
  * @brief  функция для проверки транспортного протокола, !блокирующая! только для отладки
  * @note   проверка проводится при подключенной полезной нагрузке с поданным питанием
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_dbg_tr_lvl_test(type_PN11_model* pn11_ptr)
{
	uint8_t test_data[16]={0x55, 0x00, 0x00, 0x00}, receive_data[16]={0};
	//
	pn_11_pwr_on(pn11_ptr);
	HAL_Delay(4000);
	//
	pn_11_output_set(pn11_ptr, PN11_OUTPUT_FPGA_ON);
	HAL_Delay(8000);
	printf("Output 0x%02X, input 0x%02X\n", pn_11_get_outputs_state(pn11_ptr), pn_11_get_inputs_state(pn11_ptr));
	///***  проверка uart ***///
	test_data[1] = 0x00;
	test_data[3] = crc8_rmap_data(test_data, 3);
	HAL_UART_Transmit_IT(pn11_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn11_ptr->interface.tr_lvl.huart, receive_data, 4, 100);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	//
	test_data[1] = 0x01;
	test_data[3] = crc8_rmap_data(test_data, 3);
	HAL_UART_Transmit_IT(pn11_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn11_ptr->interface.tr_lvl.huart, receive_data, 4, 100);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	//
	test_data[1] = 0x02;
	test_data[3] = crc8_rmap_data(test_data, 3);
	HAL_UART_Transmit_IT(pn11_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn11_ptr->interface.tr_lvl.huart, receive_data, 4, 100);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	//
	test_data[1] = 0x20;
	test_data[3] = crc8_rmap_data(test_data, 3);
	HAL_UART_Transmit_IT(pn11_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn11_ptr->interface.tr_lvl.huart, receive_data, 4, 100);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	//
	HAL_Delay(1000);
	pn_11_pwr_off(pn11_ptr);
}
