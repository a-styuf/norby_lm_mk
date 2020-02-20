/**
  ******************************************************************************
  * @file           : pn11.c
  * @version        : v1.0
  * @brief          : програмная модель для работы с ПН1.1А и ПН1.1Б ! без контроля питания !
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn11.h"

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
		//инициализация UART
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
	// установка канала управления температурой
	pn11_ptr->tmp_ch = tmp_ch_ptr;
	// инициализация интерфейса общения
	app_lvl_init(&pn11_ptr->interface, huart);
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
	pn11_ptr->report.temp = __REV16(pn11_ptr->tmp_ch->temp);
	pn11_ptr->report.voltage = __REV16(pn11_ptr->pwr_ch->ina226.voltage);
	pn11_ptr->report.current = __REV16(pn11_ptr->pwr_ch->ina226.current);
	pn11_ptr->report.outputs = pn_11_get_outputs_state(pn11_ptr);
	pn11_ptr->report.inputs = pn_11_get_inputs_state(pn11_ptr);
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

void pn_11_get_pwr_val (type_PN11_model* pn11_ptr)
{

}

void pn_11_get_interrupt (type_PN11_model* pn11_ptr)
{

}
