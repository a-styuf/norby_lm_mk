/**
  ******************************************************************************
  * @file           : pwr_ch.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX для удобного управления отдельным каналом питания
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pwr_ch.h"

/**
  * @brief  установка границ допустимых параметров питания
  * @param  pwr_ch_ptr: структура управления каналом питания
  * @param  i2c_ptr: верхняя граница питания
  * @param  i2c_addr: нижняя граница напряжения граница питания
  * @param  power_lim_Wt: верхняя граница мощности для сигнала Alert, не для проверки состояния канала (служит для вызова callback быстрой обработки проблемы)
  * @param  ena1_bank: банк GPIO для первого сигнала включения питания
  * @param  ena1_pos: номер GPIO в банке для первого сигнала включения питания
  * @param  ena2_bank: -
  * @param  ena2_pos: -
  * @param  ena3_bank: -
  * @param  ena3_pos: -
	* @retval 1 - успегаеая инициализация ina226, 0 - ошибка инициализации ina226
  */
int8_t pwr_ch_init(type_PWR_CHANNEL* pwr_ch_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t i2c_addr, uint16_t power_lim_Wt, GPIO_TypeDef* ena1_bank, uint16_t ena1_pos, GPIO_TypeDef* ena2_bank, uint16_t ena2_pos, GPIO_TypeDef* ena3_bank, uint16_t ena3_pos)
{
	pwr_ch_ptr->ena[0] = gpio_parameters_set(ena1_bank, ena1_pos);
	pwr_ch_ptr->ena[1] = gpio_parameters_set(ena2_bank, ena2_pos);
	pwr_ch_ptr->ena[2] = gpio_parameters_set(ena3_bank, ena3_pos);
	pwr_ch_ptr->mode = PWR_CH_OFF;
	pwr_ch_set_bound(pwr_ch_ptr, 0, 0, 0, 0);
	return ina226_init(&pwr_ch_ptr->ina226, i2c_ptr, i2c_addr, power_lim_Wt);
}

/**
  * @brief  установка границ допустимых параметров питания
  * @param  pwr_ch_ptr: структура управления каналом питания
  * @param  u_max: верхняя граница питания
  * @param  u_min: нижняя граница напряжения граница питания
  * @param  pow_max: верхняя граница мощности
  * @param  pow_min: нижняя граница мощности
  */
void pwr_ch_set_bound(type_PWR_CHANNEL* pwr_ch_ptr, float u_max, float u_min, float pow_max, float pow_min)
{
	pwr_ch_ptr->power_max = pow_max;
	pwr_ch_ptr->power_min = pow_min;
	pwr_ch_ptr->voltage_max = u_max;
	pwr_ch_ptr->voltage_min = u_min;
}

/**
  * @brief  проверка параметров питания
  * @param  pwr_ch_ptr: структура управления каналом питания
  * @param  error: тип ошибки питания
  * @retval 1 - изменилось состояние питания, 0 - ошибки остались, какие и были
  */
uint8_t pwr_ch_get_error(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t *error)
{
	uint8_t report = 0, retval = 0;
	float voltage = 0, current = 0, power = 0;
	voltage = pwr_ch_ptr->ina226.voltage/256.;
	current = pwr_ch_ptr->ina226.current/256.;
	power = voltage*current;
	if (pwr_ch_ptr->mode == PWR_CH_OFF){
		if (voltage > pwr_ch_ptr->voltage_min){
			// #ifdef DEBUG
			// 	printf("switch_error voltage %.1f\n", power);
			// #endif
			report|= PWR_CH_ERR_SWITCH;
		}
		if (power > pwr_ch_ptr->power_min){
			// #ifdef DEBUG
			// 	printf("switch_error power %.1f\n", power);
			// #endif
			report|=  PWR_CH_ERR_SWITCH;
		}
	}
	else if(pwr_ch_ptr->mode == PWR_CH_ON){
		if ((voltage <= pwr_ch_ptr->voltage_min) ||(voltage > pwr_ch_ptr->voltage_max) ){
			// #ifdef DEBUG
			// 	printf("voltage_error %.1f\n", voltage);
			// #endif
			report|= PWR_CH_ERR_VOLTAGE;
		}
		if (power > pwr_ch_ptr->power_max){
			// #ifdef DEBUG
			// 	printf("power_error %.1f\n", power);
			// #endif
			report|=  PWR_CH_ERR_PWR;
		}
		if (power <= pwr_ch_ptr->power_min){
			// #ifdef DEBUG
			// 	printf("power_error %.1f\n", power);
			// #endif
			report|=  PWR_CH_ERR_SWITCH;
		}
	}
	 //для определения изменения значения с нуля на 1 используется для old и new: (old^new)&new
	if ((((pwr_ch_ptr->error ^ report) & report)) & 0x0F){
		retval = 1;
	}
	pwr_ch_ptr->error = report;
	*error = report;
	return retval;
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
		pwr_ch_ptr->mode = PWR_CH_ON;
	}
	else{
		gpio_set(&pwr_ch_ptr->ena[0], 0);
		gpio_set(&pwr_ch_ptr->ena[1], 0);
		gpio_set(&pwr_ch_ptr->ena[2], 0);
		pwr_ch_ptr->mode = PWR_CH_OFF;
	}
}

/**
  * @brief  включение/отключение отдельных каналов питания (используется для отладки!)
  * @param  pwr_ch_ptr: структура управления каналом питания
  * @param  mask: для управления используется первые три бита: 1 - включено, 0 - выключено
  */
void pwr_ch_on_off_separatly(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t mode) //работает для всех каналов, кроме 0-го (МС) - он всегда включен
{
	gpio_set(&pwr_ch_ptr->ena[0], (mode >> 0) & 0x01);
	gpio_set(&pwr_ch_ptr->ena[1], (mode >> 1) & 0x01);
	gpio_set(&pwr_ch_ptr->ena[2], (mode >> 2) & 0x01);
}
