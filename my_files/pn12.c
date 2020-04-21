/**
  ******************************************************************************
  * @file           : pn12.c
  * @version        : v1.0
  * @brief          : програмная модель для работы с ПН1.2 (Должна совпадаться с ПН1.1 процентов на 90%)
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn12.h"
#include <stdio.h>

/**
  * @brief  инийиализация полезной нагрузки 1.2
  * @param  pn12_ptr: указатель на структуру управления ПН1.2
  */
void pn_12_init(type_PN12_model* pn12_ptr, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart)
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
	// инициализация интерфейса общения
	app_lvl_init(&pn12_ptr->interface, huart);
}

/**
  * @brief  установка различных статусов и счетчиков в значение по умолчанию для старта работы с ПН
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_reset_state(type_PN12_model* pn12_ptr)
{
	pn12_ptr->status = 0;
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
  * @brief  установка выходов управления ПН в значение по умолчанию
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_report_create(type_PN12_model* pn12_ptr)
{
	//
	memset((uint8_t*)&pn12_ptr->report, 0xFE, sizeof(type_PN12_report));
	//
	pn12_ptr->report.status 			= pn12_ptr->status;
	pn12_ptr->report.error_flags 	= pn12_ptr->error_flags;
	pn12_ptr->report.err_cnt 			= pn12_ptr->error_cnt;
	pn12_ptr->report.gap 					= 0xFE;
	pn12_ptr->report.voltage 			= pn12_ptr->pwr_ch->ina226.voltage;
	pn12_ptr->report.current 			= pn12_ptr->pwr_ch->ina226.current;
	pn12_ptr->report.temp 				= pn12_ptr->tmp_ch->temp;
	pn12_ptr->report.outputs 			= pn_12_get_outputs_state(pn12_ptr);
	pn12_ptr->report.inputs 			= pn_12_get_inputs_state(pn12_ptr);
	pn12_ptr->report.rsrv[0] 			= 0xFEFE;
	pn12_ptr->report.rsrv[1] 			= 0xFEFE;
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

/**
  * @brief  включение питания ПН1.2
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_pwr_on(type_PN12_model* pn12_ptr)
{
	pwr_ch_on_off(pn12_ptr->pwr_ch, 0x01);
}

/**
  * @brief  отключение питания ПН1.2
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_pwr_off(type_PN12_model* pn12_ptr)
{
	pwr_ch_on_off(pn12_ptr->pwr_ch, 0x00);
}


/**
  * @brief  функция для проверки переферии ПН12, !блокирующая! только для отладки
  * @note   проверка проводится при выходе информационного интерфейса с выхода на вход, при ИКУ подключенных к ТМ (по возможности)
	* 				оставшиеся сигналы ТМ подключаются по желанию
  * @param  pn12_ptr: указатель на структуру управления ПН1.1
  */
void pn_12_dbg_reset_state(type_PN12_model* pn12_ptr)
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
