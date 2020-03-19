/**
  ******************************************************************************
  * @file           : pl_cyclogram.c
  * @version        : v1.0
  * @brief          : работа с ПН и циклограммами
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pl_cyclogram.h"

/**
  * @brief  инициализация полезных нагрузок, связывание их с физическими сущностями
  * @param  pl_ptr: структура управления всеми полезными нагрузками и циклограммами
  * @param  pwr_arr: указатель на массив каналов управления питанием, в котором есть каналы для ПН
  * @param  tmp_arr: указатель на массив каналов мониторинга температуры, в котором есть каналы для ПН
  */
void pl_init(type_PL* pl_ptr, type_PWR_CHANNEL* pwr_arr, type_TMP1075_DEVICE* tmp_arr, UART_HandleTypeDef* huartA, UART_HandleTypeDef* huartB)
{
	//инициализация ПН1.1А
	pn_11_init(&pl_ptr->_11A, PL11A, &pwr_arr[1], &tmp_arr[1], huartA);
	//инициализация ПН1.1Б
	pn_11_init(&pl_ptr->_11B, PL11B, &pwr_arr[2], &tmp_arr[2], huartB);
}

/**
  * @brief  получение короткого отчета по ПН
  * @param  pl_ptr: структура управления всеми полезными нагрузками и циклограммами
  * @param  pl_num: номер полезной нагрузки соглансо списку: // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
  * @param  report: указатель на массив для отчета
  * @param  len: указатель на длину отчета
  */
void pl_report_get(type_PL* pl_ptr, uint8_t pl_num, uint8_t* report, uint8_t* len)
{
	switch(pl_num){
		case 1: //ПН1.1А
			pn_11_report_create(&pl_ptr->_11A);
			memcpy(report, &pl_ptr->_11A.report, sizeof(type_PN11_report));
			*len = sizeof(pl_ptr->_11A.report);
			break;
		case 2:
			pn_11_report_create(&pl_ptr->_11B);
			memcpy(report, &pl_ptr->_11B.report, sizeof(type_PN11_report));
			*len = sizeof(pl_ptr->_11B.report);
			break;
		case 3:
		case 4:
		case 5:
		case 6:
		default:
		report[0] = 1;
		*len = 0;
		break;
	}
}

//*** Циклограммы ***//

/**
  * @brief  инициализация циклограмм
  * @param  ccl_ptr: структура управления циклограммой
  * @param  pl_ptr: структура управления ПН-ми
  */
void cyclogram_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr)
{
	memset(ccl_ptr->array, 0x00, sizeof(ccl_ptr->array));
	ccl_ptr->mode = 0;
	ccl_ptr->num = 0;
	ccl_ptr->time_ms = 0;
	// Циклограмма 0: 0x00 - тестова циклограмма
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 0, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 1, pl_pn11A_check_temp, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 2, pl_pn11A_pwr_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 3, pl_pn11A_fpga_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 4, pl_pn11A_fpga_mcu_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 5, pl_pn11A_get_and_check_hw_telemetry, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 6, pl_pn11A_check_INT, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 7, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 8, pl_pn11A_pwr_off, 1000);
	// Циклограмма 1: 0x01 - ПН1.1А
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 0, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 1, pl_pn11A_check_temp, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 2, pl_pn11A_pwr_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 3, pl_pn11A_fpga_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 4, pl_pn11A_fpga_mcu_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 5, pl_pn11A_get_and_check_hw_telemetry, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 6, pl_pn11A_check_INT, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 7, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, 8, pl_pn11A_pwr_off, 1000);
}

/**
  * @brief  инициализация шага циклограммы
  * @param  ccl_ptr: структура управления циклограммой
  * @param  pl_ptr:  структура управления ПН-ми
  * @param  cyclogramm:  номер циклограммы
  * @param  step:  номер шага циклограммы
  * @param  (*function)(type_PL*):  функция, выполняемая на step-шаге
  * @param  delay:  задержка в мс до следующего шага
  */
void cyclogram_step_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogramm, uint8_t step, int8_t (*function)(type_PL*), uint32_t delay)
{
	ccl_ptr->array[cyclogramm].step[step].function = function;
	ccl_ptr->array[cyclogramm].step[step].state = 0;
	ccl_ptr->array[cyclogramm].step[step].delay_to_next_step_ms = delay;
}

/**
  * @brief  старт работы с циклограммами
  * @param  ccl_ptr: структура управления циклограммой
  * @param  mode:  режим циклограммы: 0-отключить циклограмму, 1-разовый проход циклограммы, 2-циклический режим
  * @param  cyclogram_num:  номер циклограммы
  * @retval статус ошибки: >0 - ок, меньше нуля - не ок
  */
int8_t cyclogram_start(type_CYCLOGRAM* ccl_ptr, uint8_t mode, uint8_t cyclogram_num)
{
	ccl_ptr->mode = mode;
	ccl_ptr->num = cyclogram_num;
	ccl_ptr->time_ms = 0;
	cyclogram_single_init(ccl_ptr, cyclogram_num);
	return 0;
}

/**
  * @brief  инициализация отдельной циклограммы перед началом работы
  * @param  ccl_ptr: структура управления циклограммой
  * @param  cyclogram_num:  номер циклограммы
  */
void cyclogram_single_init(type_CYCLOGRAM* ccl_ptr, uint8_t cyclogram_num)
{
	uint8_t i;
	ccl_ptr->array[cyclogram_num].step_timeout = 0;
	ccl_ptr->array[cyclogram_num].step_num = 0;
	ccl_ptr->array[cyclogram_num].state = 0;
	for (i=0; i<STEP_NUM; i++){
		ccl_ptr->array[cyclogram_num].step[i].state = 0;
	}
}

/**
  * @brief  обработка состояния циклограммы один раз в 100мс
  * @param  pl_ptr: структура управления циклограммой и ПН-ми
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t cyclogram_process_100ms(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr)
{
	uint8_t c_num, s_num;
	c_num = ccl_ptr->num;
	s_num = ccl_ptr->array[c_num].step_num;
	//
	ccl_ptr->time_ms += 100;
	
	//
	if (ccl_ptr->mode != 0){
		if((ccl_ptr->array[c_num].step[s_num].function != 0) && (ccl_ptr->array[c_num].step_num < STEP_NUM)){
			if (ccl_ptr->array[c_num].step[s_num].state == 0){ //шаг циклограммы не запущен
				ccl_ptr->array[c_num].step[s_num].state = ccl_ptr->array[c_num].step[s_num].function(pl_ptr);
				ccl_ptr->array[c_num].step_timeout = ccl_ptr->array[c_num].step[s_num].delay_to_next_step_ms;
			} 
			else{
				if(ccl_ptr->array[c_num].step_timeout <= 0){
					ccl_ptr->array[c_num].step_num += 1;
				}
				else{
					ccl_ptr->array[c_num].step_timeout -= 100;
				}
			}
		}
		else{
			ccl_ptr->array[c_num].step_num = 0;
			if (ccl_ptr->mode == 1){
				ccl_ptr->mode = 0;
			}		
			else if (ccl_ptr->mode == 2){
				ccl_ptr->num += 1;
				if (ccl_ptr->num >= CYCLEGRAMM_NUM){
					ccl_ptr->num = 0;
				}
				
			}
		}
		return 1;
	}
	else{
		ccl_ptr->num = 0;
		ccl_ptr->array[c_num].step_num = 0;
		return 0;
	}
}

int8_t pl_pn11A_set_iku_default(type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11A, PN11_OUTPUT_DEFAULT);
	return 1;
}

int8_t pl_pn11A_check_temp(type_PL* pl_ptr)
{
	
	return 1;
}

int8_t pl_pn11A_pwr_on(type_PL* pl_ptr)
{
	pn_11_pwr_on(&pl_ptr->_11A);
	return 1;
}

int8_t pl_pn11A_pwr_off(type_PL* pl_ptr)
{
	pn_11_pwr_off(&pl_ptr->_11A);
	return 1;
}

int8_t pl_pn11A_fpga_on(type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11A, PN11_OUTPUT_FPGA_ON);
	return 1;
}

int8_t pl_pn11A_fpga_mcu_on(type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11A, PN11_OUTPUT_FPGA_MCU_ON);
	return 1;
}

int8_t pl_pn11A_get_and_check_hw_telemetry(type_PL* pl_ptr)
{
	pn_11_get_outputs_state(&pl_ptr->_11A);
	return 1;
}

int8_t pl_pn11A_check_INT(type_PL* pl_ptr)
{
	return 1;
}
