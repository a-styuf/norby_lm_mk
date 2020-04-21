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
  * @brief  инийиализация полезной нагрузки 1.2
  * @param  pn20_ptr: указатель на структуру управления ПН1.2
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
	app_lvl_init(&pn20_ptr->interface, huart);
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


/**
  * @brief  функция для проверки переферии ПН20, !блокирующая! только для отладки
  * @note   проверка проводится при выходе информационного интерфейса с выхода на вход, при ИКУ подключенных к ТМ (по возможности)
	* 				оставшиеся сигналы ТМ подключаются по желанию
  * @param  pn20_ptr: указатель на структуру управления ПН1.1
  */
void pn_20_dbg_reset_state(type_PN20_model* pn20_ptr)
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
	HAL_UART_Transmit_IT(pn20_ptr->interface.tr_lvl.huart, test_data, 4);
	HAL_UART_Receive(pn20_ptr->interface.tr_lvl.huart, receive_data, 4, 200);
	printf("Test UART:\n");
	printf("rx_data: ");
	printf_buff(test_data, 4, '\t');
	printf("tx_data: ");
	printf_buff(receive_data, 4, '\n');
	HAL_Delay(2000);
}
