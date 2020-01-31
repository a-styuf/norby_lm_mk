#include "lm.h"

//*** LM ***//
void lm_init(type_LM_DEVICE* lm_ptr)
{
	// перезапускаем премя модуля
	lm_ptr->global_time_s = 0;
	//инициализируем питание
	pwr_init(&lm_ptr->pwr, &hi2c3);
	//инициализируем измерение температуры
	tmp_init(&lm_ptr->tmp, &hi2c2);
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
	//установка урпавляющих переменных
	pwr_ptr->ch_read_queue = 0;
	memset(&pwr_ptr->report, 0x00, 32);
	// установка парамеров отдельных каналов
	// 0 - МС
	pwr_ptr->ch[0].ena[0] = gpio_parameters_set(GPIOC, 0); // NU - всегда включено: установил такое-же как для ena[2] унификации програмной модели канала
	pwr_ptr->ch[0].ena[1] = gpio_parameters_set(GPIOC, 0); // NU - всегда включено: установил такое-же как для ena[2] унификации програмной модели канала
	pwr_ptr->ch[0].ena[2] = gpio_parameters_set(GPIOC, 0);
	ina226_init(&pwr_ptr->ch[0].ina226, hi2c_ptr, 0x40, 3);
	// 1 - ПН1.1A
	pwr_ptr->ch[1].ena[0] = gpio_parameters_set(GPIOF, 0);
	pwr_ptr->ch[1].ena[1] = gpio_parameters_set(GPIOF, 1);
	pwr_ptr->ch[1].ena[2] = gpio_parameters_set(GPIOF, 2);
	ina226_init(&pwr_ptr->ch[1].ina226, hi2c_ptr, 0x41, 5);
	// 2 - ПН1.1Б
	pwr_ptr->ch[2].ena[0] = gpio_parameters_set(GPIOF, 3);
	pwr_ptr->ch[2].ena[1] = gpio_parameters_set(GPIOF, 4);
	pwr_ptr->ch[2].ena[2] = gpio_parameters_set(GPIOF, 5);
	ina226_init(&pwr_ptr->ch[2].ina226, hi2c_ptr, 0x42, 8);
	// 3 - ПН1.2
	pwr_ptr->ch[3].ena[0] = gpio_parameters_set(GPIOF, 6);
	pwr_ptr->ch[3].ena[1] = gpio_parameters_set(GPIOF, 7);
	pwr_ptr->ch[3].ena[2] = gpio_parameters_set(GPIOF, 8);
	ina226_init(&pwr_ptr->ch[3].ina226, hi2c_ptr, 0x43, 8);
	// 4 - ПН2.0
	pwr_ptr->ch[4].ena[0] = gpio_parameters_set(GPIOF, 9);
	pwr_ptr->ch[4].ena[1] = gpio_parameters_set(GPIOF, 10);
	pwr_ptr->ch[4].ena[2] = gpio_parameters_set(GPIOF, 11);
	ina226_init(&pwr_ptr->ch[4].ina226, hi2c_ptr, 0x44, 15);
	// 5 - ПН_ДКР1
	pwr_ptr->ch[5].ena[0] = gpio_parameters_set(GPIOF, 12);
	pwr_ptr->ch[5].ena[1] = gpio_parameters_set(GPIOF, 13);
	pwr_ptr->ch[5].ena[2] = gpio_parameters_set(GPIOF, 13); // NU - всегда включено: установил такое-же как для ena[1] унификации програмной модели канала
	ina226_init(&pwr_ptr->ch[5].ina226, hi2c_ptr, 0x45, 3);
	// 6 - ПН_ДКР2
	pwr_ptr->ch[6].ena[0] = gpio_parameters_set(GPIOF, 14);
	pwr_ptr->ch[6].ena[1] = gpio_parameters_set(GPIOF, 15);
	pwr_ptr->ch[6].ena[2] = gpio_parameters_set(GPIOF, 15); // NU - всегда включено: установил такое-же как для ena[1] унификации програмной модели канала
	ina226_init(&pwr_ptr->ch[6].ina226, hi2c_ptr, 0x46, 3);
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
  * @brief  организация работы в 100ms слотах
  * @param  pwr_ptr: структура управления питанием
  */
void pwr_process_100ms(type_PWR_CONTROL* pwr_ptr)
{
	pwr_ptr->ch_read_queue += 1;
	if (pwr_ptr->ch_read_queue >= 7){
		//запускаем новые измерения
		pwr_ptr->ch_read_queue = 0;
		//создаем отчет о системе питания
		pwr_create_report(pwr_ptr);
	}
	ina226_start_read_queue(&pwr_ptr->ch[pwr_ptr->ch_read_queue].ina226);
	
}

/**
  * @brief  создание отчета для выкладывания в телеметрию
  * @param  pwr_ptr: структура управления питанием
  * @note 16-ти битные данные переварачиваются, так-как в памяти микроконтроллера они хронятся младшим байтом вперед
  */
void pwr_create_report(type_PWR_CONTROL* pwr_ptr)
{
	uint8_t i=0, ena0, ena1, ena2, gd, alert;
	volatile uint32_t gpio_state = 0;
	//отчет по GPIO
	for (i=0; i<7; i++){
		ena0 = gpio_get(&pwr_ptr->ch[i].ena[0]);
		ena1 = gpio_get(&pwr_ptr->ch[i].ena[1]);
		ena2 = gpio_get(&pwr_ptr->ch[i].ena[2]);
		gpio_state |=  ((ena2 << 2)|(ena1 << 1)|(ena0 << 0)) << (i*3);
	}
	gd = gpio_get(&pwr_ptr->gd);
	alert = gpio_get(&pwr_ptr->alert);
	gpio_state |= ((gd << 1)|(alert << 0)) << 30;
	pwr_ptr->report.gpio_state = __REV(gpio_state);
	//отчет по параметрам питания
	// 0 - МС
	pwr_ptr->report.lm_voltage = __REV16(pwr_ptr->ch[0].ina226.voltage);
	pwr_ptr->report.lm_current = __REV16(pwr_ptr->ch[0].ina226.current);
	// 1 - ПН1.1A
	pwr_ptr->report.pl11a_voltage = __REV16(pwr_ptr->ch[1].ina226.voltage);
	pwr_ptr->report.pl11a_current = __REV16(pwr_ptr->ch[1].ina226.current);
	// 2 - ПН1.1Б
	pwr_ptr->report.pl11b_voltage = __REV16(pwr_ptr->ch[2].ina226.voltage);
	pwr_ptr->report.pl11b_current = __REV16(pwr_ptr->ch[2].ina226.current);
	// 3 - ПН1.2
	pwr_ptr->report.pl12_voltage = __REV16(pwr_ptr->ch[3].ina226.voltage);
	pwr_ptr->report.pl12_current = __REV16(pwr_ptr->ch[3].ina226.current);
	// 4 - ПН2.0
	pwr_ptr->report.pl20_voltage = __REV16(pwr_ptr->ch[4].ina226.voltage);
	pwr_ptr->report.pl20_current = __REV16(pwr_ptr->ch[4].ina226.current);
	// 5 - ПН_ДКР1
	pwr_ptr->report.pl_dcr1_voltage = __REV16(pwr_ptr->ch[5].ina226.voltage);
	pwr_ptr->report.pl_dcr1_current = __REV16(pwr_ptr->ch[5].ina226.current);
	// 6 - ПН_ДКР2
	pwr_ptr->report.pl_dcr2_voltage = __REV16(pwr_ptr->ch[6].ina226.voltage);
	pwr_ptr->report.pl_dcr2_current = __REV16(pwr_ptr->ch[6].ina226.current);
}

void pwr_alert_gd_it_process(type_PWR_CONTROL* pwr_ptr, uint16_t it_position)
{
	if (it_position & (1 << pwr_ptr->gd.position)){
		pwr_ptr->gd.state = gpio_get(&pwr_ptr->gd);
	}
	if (it_position & (1 << pwr_ptr->alert.position)){
		pwr_ptr->alert.state = gpio_get(&pwr_ptr->alert);
	}
}

//*** управление измерением температуры ***//
void tmp_init(type_TMP_CONTROL* tmp_ptr, I2C_HandleTypeDef* hi2c_ptr)
{
	// установка общих шин
	tmp_ptr->alert = gpio_parameters_set(GPIOB, 9);
	// установка параметров отдельных каналов
	// 0 - МС
	tmp1075_init(&tmp_ptr->tmp1075[0], hi2c_ptr, 0x4F); 
	// 1 - ПН1.1А
	tmp1075_init(&tmp_ptr->tmp1075[1], hi2c_ptr, 0x4C); 
	// 2 - ПН1.1B
	tmp1075_init(&tmp_ptr->tmp1075[2], hi2c_ptr, 0x4D); 
	// 3 - ПН1.2
	tmp1075_init(&tmp_ptr->tmp1075[3], hi2c_ptr, 0x4A); 
	// 4 - ПН2.0
	tmp1075_init(&tmp_ptr->tmp1075[4], hi2c_ptr, 0x40); 
	
}

/**
  * @brief  создание отчета для выкладывания в телеметрию
  * @param  tmp_ptr: структура управления измерением температуры
  * @note 16-ти битные данные переварачиваются, так-как в памяти микроконтроллера они хронятся младшим байтом вперед
  */
void tmp_create_report(type_TMP_CONTROL* tmp_ptr)
{
	uint8_t i=0, alert;
	volatile uint32_t gpio_state = 0;
	//отчет по GPIO
	alert = gpio_get(&tmp_ptr->alert);
	tmp_ptr->report.gpio_state = alert;
	//отчет по параметрам температуры
	for(i=0; i<5; i++){
		tmp_ptr->report.temperature[i] = tmp_ptr->tmp1075[i].temp;
	}
}

/**
  * @brief  организация работы в 100ms слотах
  * @param  tmp_ptr: структура управления измерением температуры
  */
void tmp_process_100ms(type_TMP_CONTROL* tmp_ptr)
{
	tmp_ptr->ch_read_queue += 1;
	if (tmp_ptr->ch_read_queue >= 5){
		//запускаем новые измерения
		tmp_ptr->ch_read_queue = 0;
		//создаем отчет о системе питания
		tmp_create_report(tmp_ptr);
	}
	tmp1075_start_read_queue(&tmp_ptr->tmp1075[tmp_ptr->ch_read_queue]);	
}

/**
  * @brief  обработка прерывания по шине ALERT
  * @param  tmp_ptr: структура управления измерением температуры
  */
void tmp_alert_it_process(type_TMP_CONTROL* tmp_ptr, uint16_t it_position)
{
	if (it_position & (1 << tmp_ptr->alert.position)){
		tmp_ptr->alert.state = gpio_get(&tmp_ptr->alert);
	}
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
	for(i=0; i < leng; i++){
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

