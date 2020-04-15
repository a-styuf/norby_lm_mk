#include "lm.h"

//*** LM ***//
void lm_init(type_LM_DEVICE* lm_ptr)
{
	int8_t report = 0;
	printf("Start init: %d\n", report);
	lm_ctrl_init(lm_ptr);
	printf("\tLM-struct init init: %d\n", report);
	//инициализируем питание
	report = pwr_init(&lm_ptr->pwr, &hi2c3);
	printf("\tPwr monitors init: %d\n", report);
	//инициализируем измерение температуры
	report = tmp_init(&lm_ptr->tmp, &hi2c2);
	printf("\tTemperature monitors init: %d\n", report);
	//PL
	pl_init(&lm_ptr->pl, lm_ptr->pwr.ch, lm_ptr->tmp.tmp1075, &huart2, &huart4, &huart6);
	printf("\tPL init %d\n", report);
	//Cyclogram
	cyclogram_init(&lm_ptr->cyclogram, &lm_ptr->pl);
	printf("\tCyclogramms init: %d\n", report);
	//interfaces init
	report = interfaces_init(&lm_ptr->interface, DEV_ID);
	printf("\tCAN init: %d\n", report);
	//ext_mem
	report = ext_mem_init(&lm_ptr->mem, &hspi2);
	printf("\tExt-mem init: %d\n", report);
	//
	printf("Finish init at %d\n", lm_ptr->ctrl.global_time_s);
}

/**
  * @brief  инициализация состояния управляющих параметров МС
	* @retval статус успешности инициализации: кол-во успешно инициализированных блоков
  */
int8_t lm_ctrl_init(type_LM_DEVICE* lm_ptr)
{
	int8_t ret_val = 0;
		// инициализируем параметры управляющей структуры
	lm_ptr->ctrl.global_time_s = 0;
	lm_ptr->ctrl.status = 0;
	lm_ptr->ctrl.error_flags = 0;
	lm_ptr->ctrl.err_cnt = 0;
	lm_ptr->ctrl.rst_cnt = 0;
	lm_ptr->ctrl.pl_status = 0;
	return ret_val;
}

/**
  * @brief  создание отчета по работе МС
  */
void lm_report_create(type_LM_DEVICE* lm_ptr)
{
	memset((uint8_t*)&lm_ptr->report, 0xFE, sizeof(type_LM_REPORT));
	// инициализируем параметры управляющей структуры
	lm_ptr->report.status 				= lm_ptr->ctrl.status;
	lm_ptr->report.error_flags 		= lm_ptr->ctrl.error_flags;
	lm_ptr->report.err_cnt 				= lm_ptr->ctrl.err_cnt;
	lm_ptr->report.rst_cnt 				= lm_ptr->ctrl.rst_cnt;
	lm_ptr->report.voltage 				= lm_ptr->pwr.ch[0].ina226.voltage;
	lm_ptr->report.current 				= lm_ptr->pwr.ch[0].ina226.current;
	lm_ptr->report.temperature 		= lm_ptr->tmp.tmp1075[0].temp;
}

//*** управление питанием ***//
/**
  * @brief  инициализация состояния питания
  * @param  pwr_ptr: структура управления питанием
  * @param  hi2c_ptr: устройство I2C для общения с монитором питания
	* @retval статус успешности инициализации: кол-во успешно инициализированных блоков
  */
int8_t pwr_init(type_PWR_CONTROL* pwr_ptr, I2C_HandleTypeDef* hi2c_ptr)
{
	int8_t report = 0;
	// установка общих шин
	pwr_ptr->gd = gpio_parameters_set(GPIOC, 11);
	pwr_ptr->alert = gpio_parameters_set(GPIOC, 10);
	//установка урпавляющих переменных
	pwr_ptr->ch_read_queue = 0;
	memset(&pwr_ptr->report, 0x00, 32);
	// установка парамеров отдельных каналов
	// 0 - МС
	report += pwr_ch_init(&pwr_ptr->ch[0], hi2c_ptr, 0x40, 3, GPIOC, 0, GPIOC, 0, GPIOC, 0); // ena0-1 - всегда включено: установил такое-же как для ena[2] 
	// 1 - ПН1.1A
	report += pwr_ch_init(&pwr_ptr->ch[1], hi2c_ptr, 0x41, 8, GPIOF, 0, GPIOF, 1, GPIOF, 2);
	// 2 - ПН1.1Б
	report += pwr_ch_init(&pwr_ptr->ch[2], hi2c_ptr, 0x42, 8, GPIOF, 3, GPIOF, 4, GPIOF, 5);
	// 3 - ПН1.2
	report += pwr_ch_init(&pwr_ptr->ch[3], hi2c_ptr, 0x43, 8, GPIOF, 6, GPIOF, 7, GPIOF, 8);
	// 4 - ПН2.0
	report += pwr_ch_init(&pwr_ptr->ch[4], hi2c_ptr, 0x44, 15, GPIOF, 9, GPIOF, 10, GPIOF, 11);
	// 5 - ПН_ДКР1
	report += pwr_ch_init(&pwr_ptr->ch[5], hi2c_ptr, 0x45, 3, GPIOF, 12, GPIOF, 13, GPIOF, 13); // ena3: установил такое-же как для ena[1]
	// 6 - ПН_ДКР2
	report += pwr_ch_init(&pwr_ptr->ch[6], hi2c_ptr, 0x46, 3, GPIOF, 14, GPIOF, 15, GPIOF, 15); // ena3: установил такое-же как для ena[1]
	//
	return report;
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
		pwr_ch_on_off(&pwr_ptr->ch[i], state);
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

/**
  * @brief  создание отчета по ключам питания
  * @param  pwr_ptr: структура управления питанием
  */
uint8_t pwr_get_pwr_switch_key(type_PWR_CONTROL* pwr_ptr)
{
	uint8_t i=0, ena;
	volatile uint8_t gpio_state = 0;
	//отчет по GPIO
	for (i=0; i<7; i++){
		ena = gpio_get(&pwr_ptr->ch[i].ena[0]);
		ena &= gpio_get(&pwr_ptr->ch[i].ena[1]);
		ena &= gpio_get(&pwr_ptr->ch[i].ena[2]);
		gpio_state |= (ena << i);
	}
	return gpio_state;
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

/**
  * @brief  обработка callback функций
  * @param  pwr_ptr: структура управления измерением температуры
  * @param  error: 1 - обработка ошибки, 0 - обработка callback окончания транзакции
  */
void pwr_cb_it_process(type_PWR_CONTROL* pwr_ptr, uint8_t error)
{
	if (error){
		ina226_error_process(&pwr_ptr->ch[pwr_ptr->ch_read_queue].ina226);
		return;
	}
	ina226_body_read_queue(&pwr_ptr->ch[pwr_ptr->ch_read_queue].ina226);
}

//*** управление измерением температуры ***//
int8_t tmp_init(type_TMP_CONTROL* tmp_ptr, I2C_HandleTypeDef* hi2c_ptr)
{
	int8_t report = 0;
	// установка общих шин
	tmp_ptr->alert = gpio_parameters_set(GPIOB, 9);
	// установка параметров отдельных каналов
	// 0 - МС
	report += tmp1075_init(&tmp_ptr->tmp1075[0], hi2c_ptr, 0x4F); 
	// 1 - ПН1.1А
	report += tmp1075_init(&tmp_ptr->tmp1075[1], hi2c_ptr, 0x4C); 
	// 2 - ПН1.1B
	report += tmp1075_init(&tmp_ptr->tmp1075[2], hi2c_ptr, 0x4D); 
	// 3 - ПН1.2
	report += tmp1075_init(&tmp_ptr->tmp1075[3], hi2c_ptr, 0x4A); 
	// 4 - ПН2.0
	report += tmp1075_init(&tmp_ptr->tmp1075[4], hi2c_ptr, 0x40); 
	return report;
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
  * @brief  организация работы в 100ms слотах: запуск псевдопотока на измерение температуры
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

/**
  * @brief  обработка callback функций
  * @param  tmp_ptr: структура управления измерением температуры
  * @param  error: 1 - обработка ошибки, 0 - обработка callback окончания транзакции
  */
void tmp_cb_it_process(type_TMP_CONTROL* tmp_ptr, uint8_t error)
{
	if (error){
		tmp1075_error_process(&tmp_ptr->tmp1075[tmp_ptr->ch_read_queue]);
		return;
	}
	tmp1075_body_read_queue(&tmp_ptr->tmp1075[tmp_ptr->ch_read_queue]);
}

//*** заполенние тми и маяка ***//
void fill_tmi_and_beacon(type_LM_DEVICE* lm_ptr)
{
	uint8_t i=0;
	// beacon
	type_LM_Beacon_Frame beacon_fr;
	frame_create_header((uint8_t*)&beacon_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_BEACON ,lm_ptr->interface.frame_num, 0x00, lm_ptr->ctrl.global_time_s);
	beacon_fr.lm_status = lm_ptr->ctrl.status;
	beacon_fr.pl_status = lm_ptr->ctrl.pl_status;
  beacon_fr.lm_temp = (lm_ptr->tmp.tmp1075[0].temp >> 8) & 0xFF;
  beacon_fr.pl_power_switches = pwr_get_pwr_switch_key(&lm_ptr->pwr);
	memset(beacon_fr.filler, 0x00, sizeof(beacon_fr.filler));
	memcpy((uint8_t*)&lm_ptr->interface.tmi_data.beacon, (uint8_t*)&beacon_fr, sizeof(beacon_fr));
	// tmi
	type_LM_TMI_Data_Frame tmi_fr;
	frame_create_header((uint8_t*)&tmi_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_TMI ,lm_ptr->interface.frame_num, 0x00, lm_ptr->ctrl.global_time_s);
 // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
	for(i=0; i<6; i++){
		tmi_fr.pl_status[i] = lm_ptr->ctrl.pl_status;
	}
  for(i=0; i<7; i++){
		tmi_fr.pwr_inf[i].voltage = (lm_ptr->pwr.ch[i].ina226.voltage >> 4) & 0xFF;
		tmi_fr.pwr_inf[i].current = (lm_ptr->pwr.ch[i].ina226.current >> 4) & 0xFF;
	}
	for(i=0; i<5; i++){
		tmi_fr.temp[i] = (lm_ptr->tmp.tmp1075[i].temp >> 8) & 0xFF;
	}
  //
  tmi_fr.pl_power_switches = pwr_get_pwr_switch_key(&lm_ptr->pwr);
  tmi_fr.iss_mem_status = lm_ptr->mem.part[PART_ISS].part_fill_volume_prc;
  tmi_fr.dcr_mem_status = lm_ptr->mem.part[PART_DCR].part_fill_volume_prc;
  tmi_fr.pl_rst_count = lm_ptr->ctrl.rst_cnt;
  tmi_fr.com_reg_lm_mode = lm_ptr->interface.cmdreg.array[1];
  tmi_fr.com_reg_pwr_on_off = *(uint16_t*)&lm_ptr->interface.cmdreg.array[2];

	memset(tmi_fr.filler, 0x00, sizeof(tmi_fr.filler));
	memcpy((uint8_t*)&lm_ptr->interface.tmi_data.tmi, (uint8_t*)&tmi_fr, sizeof(tmi_fr));
}

/**
  * @brief  заполнение общей тми для МC
  * @param  lm_ptr: структура управления для МС
  */
void fill_gen_tmi(type_LM_DEVICE* lm_ptr)
{
	type_LM_GEN_TMI_Frame gen_fr;
	lm_report_create(lm_ptr);
	pn_11_report_create(&lm_ptr->pl._11A);
	pn_11_report_create(&lm_ptr->pl._11B);
	pn_dcr_report_create(&lm_ptr->pl._dcr);
	// 
	memset((uint8_t*)&gen_fr, 0xFEFE, sizeof(type_LM_GEN_TMI_Frame));
	frame_create_header((uint8_t*)&gen_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_GEN_TMI ,lm_ptr->interface.frame_num, 0x00, lm_ptr->ctrl.global_time_s);
	memcpy(gen_fr.lm_report, (uint8_t*)&lm_ptr->report, sizeof(type_LM_REPORT));
	memcpy(gen_fr.pl11a_report, (uint8_t*)&lm_ptr->pl._11A.report, sizeof(type_PN11_report));
	memcpy(gen_fr.pl11b_report, (uint8_t*)&lm_ptr->pl._11B.report, sizeof(type_PN11_report));
	memcpy(gen_fr.pldcr_report, (uint8_t*)&lm_ptr->pl._dcr.report, sizeof(type_PNDCR_report));
	//
	memcpy((uint8_t*)&lm_ptr->interface.tmi_data.gen_tmi, (uint8_t*)&gen_fr, sizeof(gen_fr));
}

/**
  * @brief  заполнение последенго принятого пакета, статуса и запись в память для Декор
  * @param  lm_ptr: структура управления для МС
	* @note   Важно! здесь же происходит сохранение данных декор в энергонезавичимую память
  */
void fill_dcr_rx_frame(type_LM_DEVICE* lm_ptr)
{
	type_DCR_STATUS_Frame status_dcr_fr;
	type_DCR_LONG_Frame long_dcr_fr;
	uint8_t data[128] = {0}, leng;
	//
	memset((uint8_t*)&status_dcr_fr, 0xFE, sizeof(type_DCR_STATUS_Frame));
	leng = pn_dcr_get_last_status(&lm_ptr->pl._dcr, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&status_dcr_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_DCR_LAST_RX_STATUS, lm_ptr->pl._dcr.rx_status_cnt, 0x00, lm_ptr->ctrl.global_time_s);
		// сохраняем в сформированный кадр данные, полученные из модели DCR
		memcpy((uint8_t*)status_dcr_fr.status_dcr_data, data, sizeof(leng));
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.dcr_status, (uint8_t*)&status_dcr_fr, sizeof(status_dcr_fr));
	}
	//
	memset((uint8_t*)&long_dcr_fr, 0xFE, sizeof(type_DCR_LONG_Frame));
	leng = pn_dcr_get_last_frame(&lm_ptr->pl._dcr, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&long_dcr_fr.header, DEV_ID, DCR_FRAME_TYPE, DATA_TYPE_DCR_LAST_RX_FRAME, lm_ptr->pl._dcr.rx_frames_cnt, 0x00, lm_ptr->ctrl.global_time_s);
		// сохраняем в сформированный кадр данные, полученные из модели DCR
		memcpy((uint8_t*)long_dcr_fr.long_dcr_frame, data, 124);
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.dcr_frame, (uint8_t*)&long_dcr_fr, sizeof(long_dcr_fr));
		// дополнительно заполняем память Декор кадрами
		ext_mem_wr_frame_to_part(&lm_ptr->mem, (uint8_t*)&long_dcr_fr, PART_DCR);
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

void printf_time(void)
{
	RTC_TimeTypeDef time;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	printf("%d:%d:%2.3f ", time.Hours, time.Minutes, time.Seconds + time.SubSeconds*(1./(1+time.SecondFraction)));
}

