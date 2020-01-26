#include "lm.h"

//*** LM ***//
void lm_init(type_LM_DEVICE* lm_ptr)
{
	// перезапускаем премя модуля
	lm_ptr->global_time_s = 0;
	//инициализируем питание
	pwr_init(&lm_ptr->pwr, &hi2c3);
}

//*** управление питанием ***//
/**
  * @brief  инийиализация состояния питания
  * @param  pwr_ptr: структура управления питанием
  * @param  hi2c_ptr: устройство I2C для общения с монитором питания
  */
void pwr_init(type_PWR_CONTROL* pwr_ptr, I2C_HandleTypeDef* hi2c_ptr)
{
	// установка общих шин
	pwr_ptr->gd = gpio_parameters_set(GPIOC, 11);
	pwr_ptr->alert = gpio_parameters_set(GPIOC, 10);
	
	// установка парамеров отдельных каналов
	// 0 - МС
	pwr_ptr->ch[0].ena[0] = gpio_parameters_set(GPIOC, 0); // NU - всегда включено: установил такое-же как для ena[2] унификации програмной модели канала
	pwr_ptr->ch[0].ena[1] = gpio_parameters_set(GPIOC, 0); // NU - всегда включено: установил такое-же как для ena[2] унификации програмной модели канала
	pwr_ptr->ch[0].ena[2] = gpio_parameters_set(GPIOC, 0);
	ina226_init(&pwr_ptr->ch[0].ina226, hi2c_ptr, 0x40);
	// 1 - ПН1.1A
	pwr_ptr->ch[1].ena[0] = gpio_parameters_set(GPIOF, 0);
	pwr_ptr->ch[1].ena[1] = gpio_parameters_set(GPIOF, 1);
	pwr_ptr->ch[1].ena[2] = gpio_parameters_set(GPIOF, 2);
	ina226_init(&pwr_ptr->ch[1].ina226, hi2c_ptr, 0x41);
	// 2 - ПН1.1Б
	pwr_ptr->ch[2].ena[0] = gpio_parameters_set(GPIOF, 3);
	pwr_ptr->ch[2].ena[1] = gpio_parameters_set(GPIOF, 4);
	pwr_ptr->ch[2].ena[2] = gpio_parameters_set(GPIOF, 5);
	ina226_init(&pwr_ptr->ch[2].ina226, hi2c_ptr, 0x42);
	// 3 - ПН1.2
	pwr_ptr->ch[3].ena[0] = gpio_parameters_set(GPIOF, 6);
	pwr_ptr->ch[3].ena[1] = gpio_parameters_set(GPIOF, 7);
	pwr_ptr->ch[3].ena[2] = gpio_parameters_set(GPIOF, 8);
	ina226_init(&pwr_ptr->ch[3].ina226, hi2c_ptr, 0x43);
	// 4 - ПН2.0
	pwr_ptr->ch[4].ena[0] = gpio_parameters_set(GPIOF, 9);
	pwr_ptr->ch[4].ena[1] = gpio_parameters_set(GPIOF, 10);
	pwr_ptr->ch[4].ena[2] = gpio_parameters_set(GPIOF, 11);
	ina226_init(&pwr_ptr->ch[4].ina226, hi2c_ptr, 0x44);
	// 5 - ПН_ДКР1
	pwr_ptr->ch[5].ena[0] = gpio_parameters_set(GPIOF, 12);
	pwr_ptr->ch[5].ena[1] = gpio_parameters_set(GPIOF, 13);
	pwr_ptr->ch[5].ena[2] = gpio_parameters_set(GPIOF, 13); // NU - всегда включено: установил такое-же как для ena[1] унификации програмной модели канала
	ina226_init(&pwr_ptr->ch[5].ina226, hi2c_ptr, 0x45);
	// 6 - ПН_ДКР2
	pwr_ptr->ch[6].ena[0] = gpio_parameters_set(GPIOF, 14);
	pwr_ptr->ch[6].ena[1] = gpio_parameters_set(GPIOF, 15);
	pwr_ptr->ch[6].ena[2] = gpio_parameters_set(GPIOF, 15); // NU - всегда включено: установил такое-же как для ena[1] унификации програмной модели канала
	ina226_init(&pwr_ptr->ch[6].ina226, hi2c_ptr, 0x46);
}

/**
  * @brief  включение/отключение питания выбранной ПН или канала ПН
  * @param  pwr_ptr: структура управления питанием
  * @param  channel_num: номер канала для управления
  * @param  mode: 1 - включить, 0 -отключить, обрезается по (&0x01)
  */
void pwr_ch_on_off(type_PWR_CONTROL* pwr_ptr, uint8_t channel_num, uint8_t mode) //работает для всех каналов, кроме 0-го (МС) - он всегда включен
{
	if (channel_num == 0) return;
	if (mode&0x01){
		gpio_set(&pwr_ptr->ch[channel_num].ena[0], 1);
		gpio_set(&pwr_ptr->ch[channel_num].ena[1], 1);
		gpio_set(&pwr_ptr->ch[channel_num].ena[2], 1);
	}
	else{
		gpio_set(&pwr_ptr->ch[channel_num].ena[0], 0);
		gpio_set(&pwr_ptr->ch[channel_num].ena[1], 0);
		gpio_set(&pwr_ptr->ch[channel_num].ena[2], 0);
	}
}

/**
  * @brief  включение/отключение питания любого из каналов по средствам битовой маски
  * @param  pwr_ptr: структура управления питанием
  * @param  pwr_switches: переменная с флагами, по которым включается питание: 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
  */
void pwr_on_off(type_PWR_CONTROL* pwr_ptr, uint8_t pwr_switches) //работает для всех каналов, кроме 0-го (МС) - он всегда включен
{
	uint8_t i, state;
	for(i=0; i<7; i++){
		state = ((pwr_switches & (1<<i)) != 0) ? 1 : 0;
		pwr_ch_on_off(pwr_ptr, i, state);
	}
}

/**
  * @brief  получение всех параметров питания, контроль параметров для отдельного канала
  * @param  channel_num: номер канала для проверки
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int16_t pwr_сh_control(uint8_t channel_num)
{
	return 0;
}

/**
  * @brief  получение всех параметров питания, контроль параметров для всех каналов
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int16_t pwr_all_control()
{
	return 0;
}


//*** протокол для передачи через VCP ***//
uint16_t com_ans_form(uint8_t req_id, uint8_t self_id, uint8_t* seq_num, uint8_t type, uint8_t leng, uint8_t* com_data, uint8_t* ans_com)
{	
	uint16_t crc=0;
	uint8_t i=0;
	ans_com[0] = req_id & 0xFF;
	ans_com[1] = self_id & 0xFF;
	ans_com[2] = *seq_num & 0xFF;
	ans_com[3] = 0x00 & 0xFF;
	ans_com[4] = type & 0xFF;
	ans_com[5] = leng & 0xFF;
	for(i=0; i < leng; i++)
	{
		ans_com[i+6] = com_data[i];
	}
	crc = crc16_ccitt(ans_com, leng+6); // like modbus
	ans_com[leng+6] = (uint8_t)((crc>>8) & 0xFF);
	ans_com[leng+7] = (uint8_t)((crc>>0) & 0xFF);
	*seq_num+=1;
	return leng+8;
}

//*** функции общего назначения
uint32_t get_uint32_val_from_bound(uint32_t val, uint32_t min, uint32_t max) //если число внутри границ - используется оно, если нет, то ближайшая граница
{
	if (val > max) return max;
	else if (val < min) return min;
	return val;
}

