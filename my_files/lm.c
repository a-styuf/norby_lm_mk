#include "lm.h"

//*** LM ***//
void lm_init(type_LM_DEVICE* lm_ptr)
{
	type_LM_LOAD_PARAM_Frame *load_param_frame_ptr = &lm_ptr->interface.tmi_data.load_parameters;
	//
	frame_create_header((uint8_t*)&load_param_frame_ptr->header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_LOAD_PARAM ,lm_ptr->ctrl.rst_cnt, 0x00);
	//
	printf("Version: %s\n", SOFT_VERSION);
	sscanf(SOFT_VERSION, "%hu.%hu.%hu", 	&load_param_frame_ptr->version[0],
																				&load_param_frame_ptr->version[1],
																				&load_param_frame_ptr->version[2]
																				);
	//
	printf("DevId: %d\n", DEV_ID);
		//инициализация времени
	clock_init();
	printf_time();
	printf("Clock init %d s\n", clock_get_time_s());
	//
	printf_time();
	printf("Start init\n");
	//
	lm_ctrl_init(lm_ptr);
	printf("\tLM-struct init\n");
	//инициализируем питание
	load_param_frame_ptr->pwr_init_report = pwr_init(&lm_ptr->pwr, &hi2c3);
	printf("\tPwr monitors init: %d\n", load_param_frame_ptr->pwr_init_report);
	//инициализируем измерение температуры
	load_param_frame_ptr->tmp_init_report = tmp_init(&lm_ptr->tmp, &hi2c2);
	printf("\tTemperature monitors init: %d\n", load_param_frame_ptr->tmp_init_report);
	//PL
	pl_init(&lm_ptr->pl, lm_ptr->pwr.ch, lm_ptr->tmp.tmp1075, &huart2, &huart4, &huart1, &huart3, &huart6);
	printf("\tPL init\n");
	//Cyclogram
	load_param_frame_ptr->cycl_init_report = cyclogram_init(&lm_ptr->cyclogram, &lm_ptr->pl, DEV_ID);
	printf("\tCyclogramms init: %d\n", load_param_frame_ptr->cycl_init_report);
	//interfaces init
	load_param_frame_ptr->int_init_report = interfaces_init(&lm_ptr->interface, DEV_ID);
	printf("\tCAN init: %d\n", load_param_frame_ptr->int_init_report);
	//ext_mem
	load_param_frame_ptr->ext_mem_init_report = ext_mem_init(&lm_ptr->mem, &hspi2);
	printf("\tExt-mem init: %d\n", load_param_frame_ptr->ext_mem_init_report);
	//
	frame_crc16_calc((uint8_t *)load_param_frame_ptr);
	//
	printf_time();
	printf("Finish init at %d s\n\n", clock_get_time_s());
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
	clock_set_time_s(0);
	lm_ptr->ctrl.status = 0;
	lm_ptr->ctrl.error_flags = 0;
	lm_ptr->ctrl.err_cnt = 0;
	lm_ptr->ctrl.rst_cnt = 0;
	lm_ptr->ctrl.pl_status = 0;
	lm_ptr->ctrl.inhibit = 0;
	lm_ptr->cfg.rst_cnt = 0;
	lm_ptr->pl_cyclogram_stop_flag = 0;
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
	lm_ptr->report.iss_mem_wr_ptr = (lm_ptr->mem.part[PART_ISS].write_ptr) & 0xFFFF;
	lm_ptr->report.dcr_mem_wd_ptr	= (lm_ptr->mem.part[PART_DCR].write_ptr) & 0xFFFF;
}

/**
  * @brief  загрузка параметров из ПЗУ
	* @param  lm_ptr: указатель на структуру управления МС
	* @retval статус загрузки параметров: 1 - хорошо, 0 - ошибка
  */
int8_t lm_load_parameters(type_LM_DEVICE* lm_ptr)
{
	uint8_t retval = 0;
		// отдельная загрузка полетного задания для ДеКоР (обязательно до записи остальных параметров ДеКоР)
	for (uint8_t i=0; i<16; i++){
		ext_mem_rd_frame_from_part_by_addr(&lm_ptr->mem, (uint8_t*)lm_ptr->interface.dcr_interface.FlightTask_1+ i*128, i, PART_DCR_FLIGHT_TASK_1);
	}
	pn_dcr_load_can_flight_task(&lm_ptr->pl._dcr, (uint8_t*)lm_ptr->interface.dcr_interface.FlightTask_1, 1);
	for (uint8_t i=0; i<16; i++){
		ext_mem_rd_frame_from_part_by_addr(&lm_ptr->mem, (uint8_t*)lm_ptr->interface.dcr_interface.FlightTask_2+ i*128, i, PART_DCR_FLIGHT_TASK_2);
	}
	pn_dcr_load_can_flight_task(&lm_ptr->pl._dcr, (uint8_t*)lm_ptr->interface.dcr_interface.FlightTask_2, 2);
	// загрузка параметров из специальной облости памяти
	if (ext_mem_rd_param(&lm_ptr->mem, (uint8_t*)&lm_ptr->loaded_cfg_frame)){
		#ifdef DEBUG
			printf("\n"); printf_time(); printf("Load parameters\n");
		#endif
		lm_set_cfg(lm_ptr, lm_ptr->loaded_cfg_frame.lm_cfg);
		pn_11_set_cfg(&lm_ptr->pl._11A, lm_ptr->loaded_cfg_frame.pl11a_cfg);
		pn_11_set_cfg(&lm_ptr->pl._11B, lm_ptr->loaded_cfg_frame.pl11b_cfg);
		pn_12_set_cfg(&lm_ptr->pl._12, lm_ptr->loaded_cfg_frame.pl12_cfg);
		pn_20_set_cfg(&lm_ptr->pl._20, lm_ptr->loaded_cfg_frame.pl20_cfg);
		pn_dcr_set_cfg(&lm_ptr->pl._dcr, lm_ptr->loaded_cfg_frame.pldcr_cfg);
		//
		retval |= 0x01;
	}
	else{
		#ifdef DEBUG
			printf("\n"); printf_time(); printf("Load parameters Error\n");
		#endif
	}
	//
	return retval;
}

/**
  * @brief  загрузка параметров в ПЗУ
	* @param  lm_ptr: указатель на структуру управления МС
	* @retval статус успешности инициализации: кол-во успешно инициализированных блоков
  */
int8_t lm_save_parameters(type_LM_DEVICE* lm_ptr)
{
	int8_t ret_val = 0;
	memset((uint8_t*)&lm_ptr->cfg_frame_to_save, 0xFE, 128);
	// запрашиваем конфигурации всех устройст
	lm_get_cfg(lm_ptr, (uint8_t *)&lm_ptr->cfg_frame_to_save.lm_cfg);
	pn_11_get_cfg(&lm_ptr->pl._11A, (uint8_t *)&lm_ptr->cfg_frame_to_save.pl11a_cfg);
	pn_11_get_cfg(&lm_ptr->pl._11B, (uint8_t *)&lm_ptr->cfg_frame_to_save.pl11b_cfg);
	pn_12_get_cfg(&lm_ptr->pl._12, (uint8_t *)&lm_ptr->cfg_frame_to_save.pl12_cfg);
	pn_20_get_cfg(&lm_ptr->pl._20, (uint8_t *)&lm_ptr->cfg_frame_to_save.pl20_cfg);
	pn_dcr_get_cfg(&lm_ptr->pl._dcr, (uint8_t *)&lm_ptr->cfg_frame_to_save.pldcr_cfg);
	// прилипляем заголовок
	frame_create_header((uint8_t *)&lm_ptr->cfg_frame_to_save.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_LM_CONFIG, 0x00, 0x00);
	// записываем в память
	ext_mem_wr_param(&lm_ptr->mem, (uint8_t *)&lm_ptr->cfg_frame_to_save);
	//
	return ret_val;
}

/**
  * @brief  получение параметров работы прибора для сохранения в ПЗУ
  * @param  type_LM_DEVICE: указатель на структуру управления МС
  * @param  cfg: укзаатель на структуру с параметрами
  * @retval  1 - ОК, 0 - ошибка
  */
void lm_get_cfg(type_LM_DEVICE* lm_ptr, uint8_t *cfg)
{
	memset((uint8_t*)&lm_ptr->cfg, 0xFE, sizeof(type_LM_сfg));
	//
	lm_ptr->cfg.iss_wr_ptr = lm_ptr->mem.part[PART_ISS].write_ptr;
	lm_ptr->cfg.iss_rd_ptr = lm_ptr->mem.part[PART_ISS].read_ptr;
	lm_ptr->cfg.dcr_wr_ptr = lm_ptr->mem.part[PART_DCR].write_ptr;
	lm_ptr->cfg.dcr_rd_ptr = lm_ptr->mem.part[PART_DCR].read_ptr;
	lm_ptr->cfg.inhibit = lm_ptr->ctrl.inhibit;
	lm_ptr->cfg.rst_cnt = lm_ptr->ctrl.rst_cnt;
	lm_ptr->cfg.cyclogram_mode = lm_ptr->cyclogram.mode;
	lm_ptr->cfg.cyclogram_num = lm_ptr->cyclogram.num;
	lm_ptr->cfg.result_num = lm_ptr->cyclogram.result.result_num;
	//
	memcpy(cfg, (uint8_t*)&lm_ptr->cfg, sizeof(type_LM_сfg));
	//
}

/**
  * @brief  получение параметров работы прибора для сохранения в ПЗУ
  * @param  pn11_ptr: указатель на структуру управления МС
  * @param  cfg: укзаатель на структуру с параметрами 
  * @retval  1 - ОК, 0 - ошибка (заготовка под проверку валидности данных)
  */
uint8_t lm_set_cfg(type_LM_DEVICE* lm_ptr, uint8_t *cfg)
{
	//
	memcpy((uint8_t*)&lm_ptr->loaded_cfg, (uint8_t*)cfg, sizeof(type_LM_сfg));
	//
	lm_ptr->mem.part[PART_ISS].write_ptr = lm_ptr->loaded_cfg.iss_wr_ptr;
	lm_ptr->mem.part[PART_ISS].read_ptr = lm_ptr->loaded_cfg.iss_rd_ptr;
	lm_ptr->mem.part[PART_DCR].write_ptr = lm_ptr->loaded_cfg.dcr_wr_ptr;
	lm_ptr->mem.part[PART_DCR].read_ptr = lm_ptr->loaded_cfg.dcr_rd_ptr;
	lm_ptr->cyclogram.result.result_num = lm_ptr->loaded_cfg.result_num;
	lm_ptr->ctrl.rst_cnt = lm_ptr->loaded_cfg.rst_cnt + 1;

	//
	lm_set_inh(lm_ptr, lm_ptr->loaded_cfg.inhibit);
	//
	cyclogram_start(&lm_ptr->cyclogram, &lm_ptr->pl, lm_ptr->loaded_cfg.cyclogram_mode, lm_ptr->loaded_cfg.cyclogram_num);
	//
	return 1;
}


/**
  * @brief  получение короткого отчета по ПН
  * @param  lm_ptr: указатель на структуру управления МС
  * @param  pl_num: номер ПН согласно #define в pl_cuclogram.h
  * @param  inh: cостояниt inh для ПН
  */
void lm_pl_inhibit_set(type_LM_DEVICE* lm_ptr, uint8_t pl_num, uint8_t inh)
{
	switch(pl_num){
		case LM:
			lm_set_inh(lm_ptr, inh);
			break;
		case PL11A:
			pn_11_set_inh(&lm_ptr->pl._11A, inh);
			break;
		case PL11B:
			pn_11_set_inh(&lm_ptr->pl._11B, inh);
			break;
		case PL12:
			pn_12_set_inh(&lm_ptr->pl._12, inh);
			break;
		case PL20:
			pn_20_set_inh(&lm_ptr->pl._20, inh);
			break;
		case PL_DCR1:
			pn_dcr_set_inh(&lm_ptr->pl._dcr, inh);
			break;	
		default:
			break;
	}
}

/**
  * @brief  установка запрета чего-либо
  * @param  pn20_ptr: указатель на структуру управления ПН
	* @param  inh: флаги отключения функционала ПН
  */
void lm_set_inh(type_LM_DEVICE* lm_ptr, uint8_t inh)
{
	lm_ptr->ctrl.inhibit = inh;
	//
	lm_ptr->ctrl.status &= ~(LM_STATUS_INH);
	lm_ptr->ctrl.status |= ((inh << 4) & LM_STATUS_INH);
	//
}

/**
  * @brief  обертка обработчика циклограмм для ПН ИСС с учетом заполеннности памяти
  * @param  lm_ptr: указатель на структуру управления ПН
	* @param  period_ms: период вызова данной функции
  */
void lm_cyclogram_process(type_LM_DEVICE* lm_ptr, uint16_t period_ms)
{
	if (lm_ptr->ctrl.inhibit & LM_INH_CCL_MEM_CHECK){
		lm_ptr->pl_cyclogram_stop_flag = 0;
	}
	else {
		if ((lm_ptr->pl_cyclogram_stop_flag == 0) & (part_get_free_volume_in_percantage(&lm_ptr->mem.part[PART_ISS]) >= ISS_MEM_TOP_BOUND_PROCENTAGE)){
			#ifdef DEBUG
				printf("\n"); printf_time(); printf("ISS MEM FULL!\n");
			#endif
			lm_ptr->pl_cyclogram_stop_flag = 1;
		}
		else if ((lm_ptr->pl_cyclogram_stop_flag == 1) & (part_get_free_volume_in_percantage(&lm_ptr->mem.part[PART_ISS]) < ISS_MEM_BOT_BOUND_PROCENTAGE)){
			#ifdef DEBUG
				printf("\n"); printf_time(); printf("ISS MEM HAVE ENOUGH VOLUME\n");
			#endif
			lm_ptr->pl_cyclogram_stop_flag = 0;
		}
	}
	//
	cyclogram_process(&lm_ptr->cyclogram, &lm_ptr->pl, lm_ptr->pl_cyclogram_stop_flag, period_ms);
	//
	lm_ptr->ctrl.status &= ~LM_STATUS_CCLGRM;
	lm_ptr->ctrl.status |= ((lm_ptr->cyclogram.state << 8) & LM_STATUS_CCLGRM);
}

/**
  * @brief  получение общего статус а ПН
  * @param  lm_ptr: указатель на структуру управления МС
	* @retval  статус полезных нагрузок
  */
uint16_t lm_get_pl_status(type_LM_DEVICE* lm_ptr)
{
	uint16_t status = 0;
	//LM
	//
	//PL1.1А
	status |= (pn_11_get_short_status(&lm_ptr->pl._11A) & 0x01) << PL11A;
	status |= (pn_11_get_short_status(&lm_ptr->pl._11A) & 0x02) << (PL11A - 1) + 8; // сдвигаем на -1  так как статус ошибки находится во втором бите
	//PL1.1B
	status |= (pn_11_get_short_status(&lm_ptr->pl._11B) & 0x01) << PL11B;
	status |= (pn_11_get_short_status(&lm_ptr->pl._11B) & 0x02) << (PL11B - 1) + 8; // сдвигаем на -1  так как статус ошибки находится во втором бите
	//PL1.2
	status |= (pn_12_get_short_status(&lm_ptr->pl._12) & 0x01) << PL12;
	status |= (pn_12_get_short_status(&lm_ptr->pl._12) & 0x02) << (PL12 - 1) + 8; // сдвигаем на -1  так как статус ошибки находится во втором бите
	//PL2.0
	status |= (pn_20_get_short_status(&lm_ptr->pl._20) & 0x01) << PL20;
	status |= (pn_20_get_short_status(&lm_ptr->pl._20) & 0x02) << (PL20 - 1) + 8; // сдвигаем на -1  так как статус ошибки находится во втором бите
	//PL_DCR
	status |= (pn_dcr_get_short_status(&lm_ptr->pl._dcr) & 0x01) << PL_DCR1;
	status |= (pn_dcr_get_short_status(&lm_ptr->pl._dcr) & 0x02) << (PL_DCR1 - 1) + 8; // сдвигаем на -1  так как статус ошибки находится во втором бите
	//
	lm_ptr->ctrl.pl_status = status;
	return status;
}

/**
  * @brief  инициализация состояния МС
	* @param  lm_ptr: указатель на структуру управления МС
  */
void lm_reset_state(type_LM_DEVICE* lm_ptr)
{
	lm_ctrl_init(lm_ptr);
	//
	pn_11_reset_state(&lm_ptr->pl._11A);
	pn_11_reset_state(&lm_ptr->pl._11B);
	pn_12_reset_state(&lm_ptr->pl._12);
	pn_20_reset_state(&lm_ptr->pl._20);
	pn_dcr_reset_state(&lm_ptr->pl._dcr);
	//
	cyclogram_reset_state(&lm_ptr->cyclogram, &lm_ptr->pl);
	//
	ext_mem_format_part(&lm_ptr->mem, PART_ISS);
	ext_mem_format_part(&lm_ptr->mem, PART_DCR);
	lm_ptr->mem.read_ptr = 0;
	//
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
  * @brief  организация работы во времянных слотах
  * @param  pwr_ptr: структура управления питанием
	* @param  period_ms: период вызова функции
  */
void pwr_process(type_PWR_CONTROL* pwr_ptr, uint16_t period_ms)
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
	report += tmp1075_init(&tmp_ptr->tmp1075[4], hi2c_ptr, 0x48); 
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
	* @param  period_ms: период вызова функции
  */
void tmp_process(type_TMP_CONTROL* tmp_ptr, uint16_t period_ms)
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
/**
  * @brief  заполнение маяка и ТМИ
  * @param  lm_ptr: структура управления для МС
  */
void fill_tmi_and_beacon(type_LM_DEVICE* lm_ptr)
{
	uint8_t i=0;
	// beacon
	type_LM_Beacon_Frame beacon_fr;
	frame_create_header((uint8_t*)&beacon_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_BEACON ,lm_ptr->interface.frame_num, 0x00);
	beacon_fr.lm_status = lm_ptr->ctrl.status;
	beacon_fr.pl_status = lm_get_pl_status(lm_ptr);
	beacon_fr.lm_temp = (lm_ptr->tmp.tmp1075[0].temp >> 8) & 0xFF;
	beacon_fr.pl_power_switches = pwr_get_pwr_switch_key(&lm_ptr->pwr);
	//
	frame_crc16_calc((uint8_t *)&beacon_fr);
	//
	memset(beacon_fr.filler, 0x00, sizeof(beacon_fr.filler));
	memcpy((uint8_t*)&lm_ptr->interface.tmi_data.beacon, (uint8_t*)&beacon_fr, sizeof(beacon_fr));
	// tmi
	type_LM_TMI_Data_Frame tmi_fr;
	frame_create_header((uint8_t*)&tmi_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_TMI ,lm_ptr->interface.frame_num, 0x00);
	if (lm_ptr->ctrl.constant_mode){
		fill_tmi_const_mode(&tmi_fr);
	}
	else{
		// 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
		tmi_fr.pl_status[LM] = lm_ptr->ctrl.status;
		tmi_fr.pl_status[PL11A] = lm_ptr->pl._11A.status;
		tmi_fr.pl_status[PL11B] = lm_ptr->pl._11B.status;
		tmi_fr.pl_status[PL12] = lm_ptr->pl._12.status;
		tmi_fr.pl_status[PL20] = lm_ptr->pl._20.status;
		tmi_fr.pl_status[PL_DCR1] = lm_ptr->pl._dcr.status;

		for(i=0; i<7; i++){
			tmi_fr.pwr_inf[i].voltage = (lm_ptr->pwr.ch[i].ina226.voltage >> 4) & 0xFF;
			tmi_fr.pwr_inf[i].current = (lm_ptr->pwr.ch[i].ina226.current >> 4) & 0xFF;
		}
		for(i=0; i<5; i++){
			tmi_fr.temp[i] = (lm_ptr->tmp.tmp1075[i].temp >> 8) & 0xFF;
		}
		//
		tmi_fr.pl_power_switches = pwr_get_pwr_switch_key(&lm_ptr->pwr);
		tmi_fr.iss_mem_status = part_get_free_volume_in_percantage(&lm_ptr->mem.part[PART_ISS]);
		tmi_fr.dcr_mem_status = part_get_free_volume_in_percantage(&lm_ptr->mem.part[PART_DCR]);
		tmi_fr.pl_rst_count = lm_ptr->ctrl.rst_cnt;
		tmi_fr.com_reg_pwr_on_off = lm_ptr->interface.cmdreg.array[CMDREG_PL_PWR_SW];
		tmi_fr.com_reg_inh = *(uint16_t*)&lm_ptr->interface.cmdreg.array[CMDREG_PL_INH_0];
		//
		tmi_fr.iss_rd_ptr = lm_ptr->mem.part[PART_ISS].read_ptr;
		tmi_fr.iss_wr_ptr = lm_ptr->mem.part[PART_ISS].write_ptr;
		tmi_fr.iss_mem_vol = lm_ptr->mem.part[PART_ISS].full_frame_num;
		tmi_fr.dcr_rd_ptr = lm_ptr->mem.part[PART_DCR].read_ptr;
		tmi_fr.dcr_wr_ptr = lm_ptr->mem.part[PART_DCR].write_ptr;
		tmi_fr.dct_mem_vol = lm_ptr->mem.part[PART_DCR].full_frame_num;
	}
	frame_crc16_calc((uint8_t *)&tmi_fr);
	//
	memset(tmi_fr.rsrv, 0x00, sizeof(tmi_fr.rsrv));
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
	pn_12_report_create(&lm_ptr->pl._12);
	pn_20_report_create(&lm_ptr->pl._20);
	pn_dcr_report_create(&lm_ptr->pl._dcr);
	// 
	memset((uint8_t*)&gen_fr, 0xFEFE, sizeof(type_LM_GEN_TMI_Frame));
	frame_create_header((uint8_t*)&gen_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_GEN_TMI ,lm_ptr->interface.frame_num, 0x00);
	memcpy(gen_fr.lm_report, (uint8_t*)&lm_ptr->report, sizeof(type_LM_REPORT));
	memcpy(gen_fr.pl11a_report, (uint8_t*)&lm_ptr->pl._11A.report, sizeof(type_PN11_report));
	memcpy(gen_fr.pl11b_report, (uint8_t*)&lm_ptr->pl._11B.report, sizeof(type_PN11_report));
	memcpy(gen_fr.pl12_report, (uint8_t*)&lm_ptr->pl._12.report, sizeof(type_PN12_report));
	memcpy(gen_fr.pl20_report, (uint8_t*)&lm_ptr->pl._20.report, sizeof(type_PN20_report));
	memcpy(gen_fr.pldcr_report, (uint8_t*)&lm_ptr->pl._dcr.report, sizeof(type_PNDCR_report));
	//
	gen_fr.iss_rd_ptr = lm_ptr->mem.part[PART_ISS].read_ptr;
	gen_fr.iss_mem_vol = lm_ptr->mem.part[PART_ISS].full_frame_num;
	gen_fr.dcr_rd_ptr = lm_ptr->mem.part[PART_DCR].read_ptr;
	gen_fr.dct_mem_vol = lm_ptr->mem.part[PART_DCR].full_frame_num;
	//
	frame_crc16_calc((uint8_t *)&gen_fr);
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
		frame_create_header((uint8_t*)&status_dcr_fr.header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_DCR_LAST_RX_STATUS, lm_ptr->pl._dcr.status_cnt, 0x00);
		lm_ptr->pl._dcr.status_cnt += 1;
		// сохраняем в сформированный кадр данные, полученные из модели DCR
		memcpy((uint8_t*)status_dcr_fr.status_dcr_data, data, leng);
		//
		frame_crc16_calc((uint8_t *)&status_dcr_fr);
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.dcr_status, (uint8_t*)&status_dcr_fr, sizeof(status_dcr_fr));
		// сохраняем статус в память на всякий случай
		ext_mem_wr_frame_to_part(&lm_ptr->mem, (uint8_t*)&lm_ptr->interface.tmi_data.dcr_status, PART_DCR_STATUS);
	}
	//
	memset((uint8_t*)&long_dcr_fr, 0xFE, sizeof(type_DCR_LONG_Frame));
	leng = pn_dcr_get_last_frame(&lm_ptr->pl._dcr, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&long_dcr_fr.header, DEV_ID, DCR_FRAME_TYPE, DATA_TYPE_DCR_LAST_RX_FRAME, lm_ptr->pl._dcr.frame_cnt, 0x00);
		lm_ptr->pl._dcr.frame_cnt += 1;
		// сохраняем в сформированный кадр данные, полученные из модели DCR
		memcpy((uint8_t*)long_dcr_fr.long_dcr_frame, data, 124);
		//
		frame_crc16_calc((uint8_t *)&long_dcr_fr);
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.dcr_frame, (uint8_t*)&long_dcr_fr, sizeof(long_dcr_fr));
		// дополнительно заполняем память Декор кадрами
		ext_mem_wr_frame_to_part(&lm_ptr->mem, (uint8_t*)&long_dcr_fr, PART_DCR);
	}
}

/**
  * @brief  заполнение последенго принятого пакета для ПН1.1А, ПН1.1Б, ПН1.2, ПН2.0
  * @param  lm_ptr: структура управления для МС
  */
void fill_pl_iss_last_frame(type_LM_DEVICE* lm_ptr)
{
	type_PL_ISS_INT_data last_frame[4]; //0-ПН1.1А, 1-ПН1.1Б, 2-ПН1.2, 3-ПН2.0
	uint8_t data[256] = {0}, leng;
	// pl1.1a
	memset((uint8_t*)&last_frame[PL11A-1], 0x00, sizeof(type_PL_ISS_INT_data));
	memset(data, 0x00, 128);
	leng = pn_11_get_last_frame_in_128B_format(&lm_ptr->pl._11A, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&last_frame[PL11A-1].header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_PL11A_INT_DATA, 0x00, 0x00); //todo: необходимо сделать номер кадра!
		// сохраняем в сформированный кадр данные, полученные из модели ПН1.1
		memcpy((uint8_t*)&last_frame[PL11A-1].data, data, 116);
		//
		frame_crc16_calc((uint8_t*)&last_frame[PL11A-1]);
		// сохраняем данные дополнительно в переменную с запросом
		data[124] = 3;
		memcpy((uint8_t*)&lm_ptr->interface.pl_iss_interface.InstaMessage[PL11A-1][0], data, 128);
		//
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.pl11a_frame, (uint8_t*)&last_frame[PL11A-1], sizeof(type_PL_ISS_INT_data));
	}
	// pl1.1b
	memset((uint8_t*)&last_frame[PL11B-1], 0x00, sizeof(type_PL_ISS_INT_data));
	leng = pn_11_get_last_frame_in_128B_format(&lm_ptr->pl._11B, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&last_frame[PL11B-1].header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_PL11B_INT_DATA,0x00, 0x00); //todo: необходимо сделать номер кадра!
		// сохраняем в сформированный кадр данные, полученные из модели ПН1.1
		memcpy((uint8_t*)&last_frame[PL11B-1].data, data, 116);
		//
		frame_crc16_calc((uint8_t*)&last_frame[PL11A-1]);
		// сохраняем данные дополнительно в переменную с запросом
		data[124] = 3;
		memcpy((uint8_t*)&lm_ptr->interface.pl_iss_interface.InstaMessage[PL11B-1][0], data, 128);
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.pl11a_frame, (uint8_t*)&last_frame[PL11B-1], sizeof(type_PL_ISS_INT_data));
	}
	// pl1.2
	memset((uint8_t*)&last_frame[PL12-1], 0x00, sizeof(type_PL_ISS_INT_data));
	leng = pn_12_get_last_frame_in_128B_format(&lm_ptr->pl._12, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&last_frame[PL12-1].header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_PL11B_INT_DATA, 0x00, 0x00); //todo: необходимо сделать номер кадра!
		// сохраняем в сформированный кадр данные, полученные из модели ПН1.1
		memcpy((uint8_t*)&last_frame[PL12-1].data, data, 116);
		//
		frame_crc16_calc((uint8_t*)&last_frame[PL12-1]);
		// сохраняем данные дополнительно в переменную с запросом
		data[124] = 3;
		memcpy((uint8_t*)&lm_ptr->interface.pl_iss_interface.InstaMessage[PL12-1][0], data, 128);
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.pl12_frame, (uint8_t*)&last_frame[PL12-1], sizeof(type_PL_ISS_INT_data));
	}
	// pl2.0
	memset((uint8_t*)&last_frame[PL20-1], 0x00, sizeof(type_PL_ISS_INT_data));
	leng = pn_20_int_get_last_data(&lm_ptr->pl._20, data);
	if (leng) {
		// формируем заголовок для кадра
		frame_create_header((uint8_t*)&last_frame[PL20-1].header, DEV_ID, SINGLE_FRAME_TYPE, DATA_TYPE_PL20_INT_DATA, 0x00, 0x00); //todo: необходимо сделать номер кадра!
		// сохраняем в сформированный кадр данные, полученные из модели ПН1.1
		memcpy((uint8_t*)&last_frame[PL20-1].data, data, 116);
		//
		frame_crc16_calc((uint8_t*)&last_frame[PL12-1]);
		// сохраняем данные дополнительно в переменную с запросом
		data[124] = 3;
		data[127] = leng & 0xFF;
		memcpy((uint8_t*)&lm_ptr->interface.pl_iss_interface.InstaMessage[PL20-1][0], data, 128);
		// сохраняем сформированный кадр в расшаренную can-память
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.pl20_frame, (uint8_t*)&last_frame[PL20-1], sizeof(type_PL_ISS_INT_data));
	}
	//
}

/**
  * @brief  заполнение подадреса для резльтатов циклограммы + выкладывание в память
  * @param  lm_ptr: структура управления для МС
  */
void fill_pl_cyclogramm_result(type_LM_DEVICE* lm_ptr)
{
	if (lm_ptr->cyclogram.result.cyclogram_result_ready_flag){
		//сбрасываем флаг
		lm_ptr->cyclogram.result.cyclogram_result_ready_flag = 0;
		//выкладываем на подадрес
		memcpy((uint8_t*)&lm_ptr->interface.tmi_data.cyclograma_result, (uint8_t*)&lm_ptr->cyclogram.result.box, sizeof(type_PL_CYCLOGRAMA_RESULT));
		//сохраняем данные в память для результатов циклограммы
		ext_mem_wr_frame_to_part(&lm_ptr->mem, (uint8_t*)&lm_ptr->cyclogram.result.box.header, PART_ISS);
		for(uint8_t i=0; i<(lm_ptr->cyclogram.result.box.header.header.arch_len); i++){
			ext_mem_wr_frame_to_part(&lm_ptr->mem, (uint8_t*)&lm_ptr->cyclogram.result.box.body[i], PART_ISS);
		}
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

void pl_iss_get_app_lvl_reprot(uint8_t pl_type, uint8_t *instasend_data, char *report_ctr)
{
	uint8_t ctrl_byte;
	char b_data_str[128] = {0};
	uint32_t addr;
	char mode[4][8]={"0x00", "0x01", "rd", "wr"}, pl_name[8][8] = {"_", "1.1_A", "1.1_B", "1.2", "2.0"};
	if (pl_type == PL20){
		for (uint8_t i=0; i<6; i++){
			sprintf(b_data_str + 3*i, "%02X ", instasend_data[i]);
		}
		ctrl_byte = instasend_data[127];
		sprintf(report_ctr, "PL%s: cmd 0x%02X, b_len %d, data %s\n", pl_name[pl_type], instasend_data[3], ctrl_byte, b_data_str);
	}
	else{
		addr = __REV(*(uint32_t*)&instasend_data[0]);
		ctrl_byte = instasend_data[127];
		sprintf(report_ctr, "PL%s: mode %s, u32_len %d addr 0x%08X\n", pl_name[pl_type], mode[ctrl_byte>>6], (ctrl_byte&0x3F) + 1, addr);
	}
	
}
