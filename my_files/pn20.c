/**
  ******************************************************************************
  * @file           : pn20.c
  * @version        : v1.0
  * @brief          : програмная модель для работы с ПН2.0 (Должна совпадаться с ПН1.1 процентов на 70% исключая протокол общения)
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn20.h"
#include <stdio.h>

/**
  * @brief  инийиализация полезной нагрузки ПН2.0
  * @param  pn20_ptr: указатель на структуру управления ПН2.0
  */
void pn_20_init(type_PN20_model* pn20_ptr, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart)
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
	// инициализация интерфейса общения
	
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_reset_state(type_PN20_model* pn20_ptr)
{
	pn20_ptr->status = 0;
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
	pn20_ptr->report.gap 					= 0xFE;
	pn20_ptr->report.voltage 			= pn20_ptr->pwr_ch->ina226.voltage;
	pn20_ptr->report.current 			= pn20_ptr->pwr_ch->ina226.current;
	pn20_ptr->report.temp 				= pn20_ptr->tmp_ch->temp;
	pn20_ptr->report.outputs 			= pn_20_get_outputs_state(pn20_ptr);
	pn20_ptr->report.inputs 			= pn_20_get_inputs_state(pn20_ptr);
	pn20_ptr->report.rsrv[0] 			= 0xFEFE;
	pn20_ptr->report.rsrv[1] 			= 0xFEFE;
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

/**
  * @brief  включение питания ПН1.2
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_pwr_on(type_PN20_model* pn20_ptr)
{
	pwr_ch_on_off(pn20_ptr->pwr_ch, 0x01);
}

/**
  * @brief  отключение питания ПН1.2
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_pwr_off(type_PN20_model* pn20_ptr)
{
	pwr_ch_on_off(pn20_ptr->pwr_ch, 0x00);
}

///*** pl interface ***///

/**
  * @brief  инициализация интерфейса общения для ПН2.0
  * @param  pn_dcr_ptr: указатель на структуру управления ПН2.0
	* @param  uart_ptr: указатель на uart, используемый для данной ПН
  */
void pn_20_int_init(type_PN20_interface *int_ptr, UART_HandleTypeDef *uart_ptr)
{
	//привязываем конкретный uart к полезной нагрузке
	int_ptr->huart = uart_ptr;
	// обнуляем переменные состояний
	int_ptr->tx_len = 0;
	int_ptr->rx_len = 0;
	int_ptr->rx_ptr = 0;
	//запускаем первое чтение байта
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_buff + int_ptr->rx_ptr, 1);
	// чистим буферы приема и структуры кадров на всякий случай
	memset(int_ptr->tx_data, 0x00, sizeof(int_ptr->tx_data));
	memset(int_ptr->rx_data, 0x00, sizeof(int_ptr->rx_data));
	memset(int_ptr->rx_buff, 0x00, sizeof(int_ptr->rx_buff));
	memset(&int_ptr->tx_msg.array, 0x00, sizeof(type_PN20_frame));
	memset(&int_ptr->rx_msg.array, 0x00, sizeof(type_PN20_frame));
	//сбрасываем ошибки и счетчики
	_pn_20_int_error_collector(int_ptr, PN_20_ERR_NO_ERROR);
}

/**
  * @brief  отправка произвольного массива в UART
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
	* @param  data: указатель на массив данных для передачи
	* @param  len: указатель на длину данных для передачи
  */
void pn_20_int_send(type_PN20_interface *int_ptr, uint8_t* data, uint8_t len)
{
	memcpy(int_ptr->tx_data, data, len);
	int_ptr->tx_len = len;
	HAL_UART_Transmit_IT(int_ptr->huart, int_ptr->tx_data, int_ptr->tx_len);
}

/**
  * @brief  отпрака команды по формату ПН2.0
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
	* @param  data_len: длина данных для отправки (1 или 2)
	* @param  data_hb: старший байт данных для отправки
	* @param  data_lb: младший байт данных для отправки
  */
void pn_20_int_send_frame(type_PN20_interface *int_ptr, uint8_t data_len, uint8_t data_hb, uint8_t data_lb)
{
	if (int_ptr->rx_timeout == 0){
		//формируем кадр для отправки
		int_ptr->tx_msg.frame.sof 			= 0xAA;
		int_ptr->tx_msg.frame.len 			= data_len;
		int_ptr->tx_msg.frame.data_hb 	= data_hb;
		int_ptr->tx_msg.frame.data_lb 	= data_lb;
		int_ptr->tx_msg.frame.crc8 			= сrc8_calc_for_pn_20(int_ptr->tx_msg.array.frame, 4);
		int_ptr->tx_msg.frame.eot 			= 0xAA;
		//отправляем данные
		memcpy(int_ptr->tx_data, &int_ptr->tx_msg, sizeof(type_PN20_frame));
		int_ptr->tx_len = sizeof(type_PN20_frame);
		HAL_UART_Transmit_IT(int_ptr->huart, int_ptr->tx_data, int_ptr->tx_len);
		// устанавливаем таймаут на ответ
		int_ptr->rx_timeout = PN_20_UART_TIMEOUT_MS;
	}
	else{
		_pn_20_int_error_collector(int_ptr, PN_20_INTERFACE_ERR_TIMEOUT);
	}
}

/**
  * @brief  обработка колбэка на прием данных (для вызова в CB от прерывания на прием)
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_20_int_rx_huart_cb(type_PN20_interface *int_ptr)
{
	uint8_t frame_start = 0xAA, frame_end = 0x55;
	// инкрементация длины
	int_ptr->rx_ptr++;
	// проверка на окончание пакета
	if (int_ptr->rx_ptr >= 128){
		_pn_20_int_error_collector(int_ptr, PN_20_INTERFACE_ERR_BAD_FRAME);
	}
	// запрос нового байта
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_buff + int_ptr->rx_ptr, 1);
}

/**
  * @brief  обаработка таймаута на прием данных
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
	* @param  period_ms: значение периода вызова данной функции
  */
void pn_20_int_rx_timeout_cb(type_PN20_interface *int_ptr, uint16_t period_ms)
{
	// обсчитываем изменение таймаута
	if ((int_ptr->rx_timeout == 0) || (int_ptr->rx_timeout > period_ms)){
		int_ptr->rx_timeout = 0;
	}
	else{
		int_ptr->rx_timeout -= period_ms;
	}
	// обарботка принятого пакета
	
	// запрос нового байта
	HAL_UART_Receive_IT(int_ptr->huart, int_ptr->rx_buff + int_ptr->rx_ptr, 1);
}

/**
  * @brief  проверка массива на наличие в ней пакета по протоколу ПН2.0
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
	* @param  data: указатель на массив данных для проверки
	* @param  len: длина массива данных для проверки
  */
void pn_20_int_check_frame(type_PN20_interface *int_ptr, uint8_t data, uint8_t len)
{

}

/**
  * @brief  обработка колбэка на успешную отправку данных данных
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_20_int_tx_prcs_cb(type_PN20_interface *int_ptr)
{
	int_ptr->tx_cnt++;
}

/**
  * @brief  обработка колбэка на ошибку при приеме или передачи
  * @param  pn_dcr_ptr: указатель на структуру управления ПН_ДКР
  */
void pn_20_int_err_prcs_cb(type_PN20_interface *int_ptr)
{
	uint32_t HAL_UART_ERROR;
	HAL_UART_ERROR = HAL_UART_GetError(int_ptr->huart);
	if (HAL_UART_ERROR & HAL_UART_ERROR_NE)
	{
		_pn_20_int_error_collector(int_ptr, PN_20_INTERFACE_ERR_HAL);
		HAL_UART_ERROR &= ~HAL_UART_ERROR_NE;
	}
	else if (HAL_UART_ERROR){
		_pn_dcr_uart_error_collector(int_ptr, PN_20_INTERFACE_ERR_HAL);
	}
}

/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  int_ptr: указатель на структуру управления ПН_ДКР
  * @param  error: ошибка, согласно define-ам PN_DCR_UART_ERR_... в .h
  */
void  _pn_20_int_error_collector(type_PN20_interface *int_ptr, uint16_t error)
{
  switch(error){
    case PN_20_INTERFACE_ERR_NO_ERROR:
      int_ptr->error_flags = PN_20_INTERFACE_ERR_NO_ERROR;
      int_ptr->error_cnt = 0;
      break;
    case PN_20_INTERFACE_ERR_BAD_FRAME:
    case PN_20_INTERFACE_ERR_TIMEOUT:
    case PN_20_INTERFACE_ERR_HAL:
			int_ptr->error_flags = error;
      int_ptr->error_cnt += 1;
      break;
    default:
      int_ptr->error_flags |= PN_20_INTERFACE_ERR_OTHER;
      int_ptr->error_cnt += 1;
      break;
  }
}

///*** debug test ***///
/**
  * @brief  функция для проверки переферии ПН20, !блокирующая! только для отладки
  * @note   проверка проводится при выходе информационного интерфейса с выхода на вход, при ИКУ подключенных к ТМ (по возможности)
	* 				оставшиеся сигналы ТМ подключаются по желанию
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_dbg_test(type_PN20_model* pn20_ptr)
{
	uint8_t error=0, status=0, test_data[16]={0xAA, 0x55, 0x02, 0x03}, receive_data[16]={0};
		pwr_ch_on_off_separatly(pn20_ptr->pwr_ch, 0x07);
	HAL_Delay(2000);
	///***  проверка gpio ***///
	HAL_Delay(500);
	pn_20_output_set(pn20_ptr, 0x0F);
	HAL_Delay(500);
	printf("Check gpio 0xF:");
	printf(": output 0x%02X, input 0x%02X\n", pn_20_get_outputs_state(pn20_ptr), pn_20_get_inputs_state(pn20_ptr));
	//
	HAL_Delay(500);
	pn_20_output_set(pn20_ptr, 0x00);
	HAL_Delay(500);
	printf("Check gpio 0x0:");
	printf(": output 0x%02X, input 0x%02X\n", pn_20_get_outputs_state(pn20_ptr), pn_20_get_inputs_state(pn20_ptr));
	///***  проверка uart ***///
	HAL_UART_Transmit_IT(pn20_ptr->interface.huart, test_data, 4);
	HAL_UART_Receive(pn20_ptr->interface.huart, receive_data, 4, 200);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	HAL_Delay(2000);
}
