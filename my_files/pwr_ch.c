/**
  ******************************************************************************
  * @file           : pwr_ch.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX для удобного управления отдельным каналом питания
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pwr_ch.h"


int8_t pwr_ch_init(type_PWR_CHANNEL* pwr_ch_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t i2c_addr, uint16_t power_lim_Wt, GPIO_TypeDef* ena1_bank, uint16_t ena1_pos, GPIO_TypeDef* ena2_bank, uint16_t ena2_pos, GPIO_TypeDef* ena3_bank, uint16_t ena3_pos)
{
	pwr_ch_ptr->ena[0] = gpio_parameters_set(ena1_bank, ena1_pos);
	pwr_ch_ptr->ena[1] = gpio_parameters_set(ena2_bank, ena2_pos);
	pwr_ch_ptr->ena[2] = gpio_parameters_set(ena3_bank, ena3_pos);
	return ina226_init(&pwr_ch_ptr->ina226, i2c_ptr, i2c_addr, power_lim_Wt);
}

/**
  * @brief  включение/отключение питания выбранной ПН или канала ПН
  * @param  pwr_ch_ptr: структура управления каналом питания
  * @param  mode: 1 - включить, 0 -отключить, обрезается по (&0x01)
  */
void pwr_ch_on_off(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t mode) //работает для всех каналов, кроме 0-го (МС) - он всегда включен
{
	if (mode&0x01){
		gpio_set(&pwr_ch_ptr->ena[0], 1);
		gpio_set(&pwr_ch_ptr->ena[1], 1);
		gpio_set(&pwr_ch_ptr->ena[2], 1);
	}
	else{
		gpio_set(&pwr_ch_ptr->ena[0], 0);
		gpio_set(&pwr_ch_ptr->ena[1], 0);
		gpio_set(&pwr_ch_ptr->ena[2], 0);
	}
}

/**
  * @brief  включение/отключение отдельных каналов питания (используется для проверки)
  * @param  pwr_ch_ptr: структура управления каналом питания
  * @param  mask: для управления используется первые три бита: 1 - включено, 0 - выключено
  */
void pwr_ch_on_off_separatly(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t mode) //работает для всех каналов, кроме 0-го (МС) - он всегда включен
{
	gpio_set(&pwr_ch_ptr->ena[0], (mode >> 0) & 0x01);
	gpio_set(&pwr_ch_ptr->ena[1], (mode >> 1) & 0x01);
	gpio_set(&pwr_ch_ptr->ena[2], (mode >> 2) & 0x01);
}
