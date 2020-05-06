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
	// инициализация интерфейса общения
	app_lvl_init(&pn11_ptr->interface, huart);
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn11_ptr: указатель на структуру управления ПН1.1
  */
void pn_11_reset_state(type_PN11_model* pn11_ptr)
{
	pn11_ptr->status = 0;
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

/**
  * @brief  получение последнего пакета принятого интерфейсом полезной нагрузки, для выставления на подадрес CAN
  * @param  pn11_ptr: указатель на структуру управления полезной нагрузкой
  * @retval >0 длина последнего принятого пакета, 0 - пакет уже прочитан, или нулевой длины
  */
uint8_t pn_11_get_last_frame(type_PN11_model* pn11_ptr, uint8_t *data)
{
	return app_lvl_get_last_rx_frame(&pn11_ptr->interface, data);
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
	uint32_t u32_data[30], addr;
	//
	u32_len = (insta_send_data[3] & 0x3F) + 1;
	if (u32_len > 30) u32_len = 30;
	for (uint8_t i=0; i < u32_len; i++){
		u32_data[i] = __REV(*(uint32_t*)&insta_send_data[8+4*i]);  // 4 - сдвиг из-за ctrl_byte, 4 - сдвиг из-за адреса, 4*i - сдвиг указателя по 4 байта
	}
	//
	addr = __REV(*(uint32_t*)&insta_send_data[4]);
	//
	mode = insta_send_data[3] >> 6;
	//
	switch(mode){
		case APP_LVL_MODE_READ:
			pn_11_read_req_u32_data(pn11_ptr, addr, u32_len);
			insta_send_data[0] = 0x01;
		break;
		case APP_LVL_MODE_WRITE:
			pn_11_write_u32_data(pn11_ptr, addr, u32_data, u32_len);
			insta_send_data[0] = 0x01;
		break;
	}
	return 0;
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
