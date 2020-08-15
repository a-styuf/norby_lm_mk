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
void pl_init(type_PL* pl_ptr, type_PWR_CHANNEL* pwr_arr, type_TMP1075_DEVICE* tmp_arr, UART_HandleTypeDef* huart11A, UART_HandleTypeDef* huart11B, UART_HandleTypeDef* huart12, UART_HandleTypeDef* huart20, UART_HandleTypeDef* huartDCR)
{
	//инициализация ПН1.1А
	pn_11_init(&pl_ptr->_11A, PL11A, &pwr_arr[PL11A], &tmp_arr[PL11A], huart11A);
	//инициализация ПН1.1Б
	pn_11_init(&pl_ptr->_11B, PL11B, &pwr_arr[PL11B], &tmp_arr[PL11B], huart11B);
	//инициализация ПН1.2
	pn_12_init(&pl_ptr->_12, PL12, &pwr_arr[PL12], &tmp_arr[PL12], huart12);
	//инициализация ПН2.0
	pn_20_init(&pl_ptr->_20, PL20, &pwr_arr[PL20], &tmp_arr[PL20], huart20);
	//инициализация ДеКоР
	pn_dcr_init(&pl_ptr->_dcr, PL_DCR1,  huartDCR, &pwr_arr[PL_DCR1], &pwr_arr[PL_DCR2]);
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
		case PL11A: //ПН1.1А
			pn_11_report_create(&pl_ptr->_11A);
			memcpy(report, &pl_ptr->_11A.report, sizeof(type_PN11_report));
			*len = sizeof(pl_ptr->_11A.report);
			break;
		case PL11B:
			pn_11_report_create(&pl_ptr->_11B);
			memcpy(report, &pl_ptr->_11B.report, sizeof(type_PN11_report));
			*len = sizeof(pl_ptr->_11B.report);
			break;
		case PL12:
			pn_12_report_create(&pl_ptr->_12);
			memcpy(report, &pl_ptr->_12.report, sizeof(type_PN12_report));
			*len = sizeof(pl_ptr->_12.report);
			break;
		case PL20:
			pn_20_report_create(&pl_ptr->_20);
			memcpy(report, &pl_ptr->_20.report, sizeof(type_PN20_report));
			*len = sizeof(pl_ptr->_20.report);
			break;
		case PL_DCR1:
		case PL_DCR2:
			pn_dcr_report_create(&pl_ptr->_dcr);
			memcpy(report, &pl_ptr->_dcr.report, sizeof(type_PNDCR_report));
			*len = sizeof(pl_ptr->_dcr.report);
			break;
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
int8_t cyclogram_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t dev_id)
{
	memset(ccl_ptr->array, 0x00, sizeof(ccl_ptr->array));
	ccl_ptr->mode = 0;
	ccl_ptr->num = 0;
	ccl_ptr->time_ms = 0;
	ccl_ptr->state = CYCLOGRAM_STATUS_OK;
	//
	ccl_ptr->result.lm_id = dev_id;
	ccl_ptr->result.body_offset = 0;
	ccl_ptr->result.body_num = 0;
	ccl_ptr->result.tmi_slice_num = 0;
	ccl_ptr->result.result_num = 0;
	ccl_ptr->result.cyclogram_result_ready_flag = 0;
	//
	// Циклограмма 0: 0x04 - ПН2.0
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, result_init, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_pwr_on, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_set_iku_pl_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_read_sw_tmi_1, 5000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_tas_start_1, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_tas_read_1, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_tas_start_2, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_tas_read_2, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_tas_start_3, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_tas_read_3, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_sd_start_1, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_sd_read_1, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_sd_start_2, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_sd_read_2, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_sd_start_3, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_sd_read_3, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_read_sw_tmi_2, 5000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_pwr_off, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_write_result, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 0, result_finish, 100);
	//
	cyclogram_stop_step_init(ccl_ptr, pl_ptr, 0, pl_pn20_stop, 1000);
	
	// Циклограмма 1: 0x01 - ПН1.1A
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, result_init, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_pwr_on, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_fpga_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_interface_reset_and_sync, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_write_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_read_req_mode, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_read_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_fpga_mcu_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_check_and_save_tmi, 240000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_fpga_on, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_read_req_all, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_read_all, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_pwr_off, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 1, result_finish, 100);
	//
	cyclogram_stop_step_init(ccl_ptr, pl_ptr, 1, pl_pn11A_stop, 1000);

	// Циклограмма 2: 0x02 - ПН1.1B
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, result_init, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_pwr_on, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_fpga_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_interface_reset_and_sync, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_write_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_read_req_mode, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_read_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_fpga_mcu_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_check_and_save_tmi, 240000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_fpga_on, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_read_req_all, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_read_all, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_pwr_off, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 2, result_finish, 100);
	//
	cyclogram_stop_step_init(ccl_ptr, pl_ptr, 2, pl_pn11B_stop, 1000);

	// Циклограмма 3: 0x03 - ПН1.2
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, result_init, 100); 
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_interface_init, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 500); 		//s1
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_set_iku_default, 1000); 			//s2
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_pwr_on, 2000);								//s3
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 1000);		//s5
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 1000);		//s5
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 580000);	//s8
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 1000);		//s11
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_read_req_all, 5000);					//s12
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_read_all, 5000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_pwr_off, 4000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 3, result_finish, 100);
	//
	cyclogram_stop_step_init(ccl_ptr, pl_ptr, 3, pl_pn12_stop, 1000);
	
	// Циклограмма 4: 0x04 - ПН2.0
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, result_init, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_pwr_on, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_set_iku_pl_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_read_sw_tmi_1, 5000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_tas_start_1, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_tas_read_1, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_tas_start_2, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_tas_read_2, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_tas_start_3, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_tas_read_3, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_sd_start_1, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_sd_read_1, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_sd_start_2, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_sd_read_2, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_sd_start_3, 16000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_sd_read_3, 3000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_read_sw_tmi_2, 5000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_pwr_off, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_write_result, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 4, result_finish, 100);
	//
	cyclogram_stop_step_init(ccl_ptr, pl_ptr, 4, pl_pn20_stop, 1000);

	// Циклограмма 5: 0x05 - ПН1.1A и ПН1.1Б
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, result_init, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_set_iku_default, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_pwr_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_pwr_on, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_check_and_save_tmi, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_fpga_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_fpga_on, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_interface_reset_and_sync, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_interface_reset_and_sync, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11_A_B_write_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_read_req_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_read_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_read_req_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_read_mode, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_fpga_mcu_on, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_fpga_mcu_on, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_check_and_save_tmi, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_check_and_save_tmi, 240000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_fpga_on, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_fpga_on, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_read_req_all, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_read_all, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_read_req_all, 2000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_read_all, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_set_iku_default, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_set_iku_default, 100);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_pwr_off, 1000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_pwr_off, 6000);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11A_check_and_save_tmi, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, pl_pn11B_check_and_save_tmi, 500);
	cyclogram_step_init(ccl_ptr, pl_ptr, 5, result_finish, 100);
	//
	cyclogram_stop_step_init(ccl_ptr, pl_ptr, 5, pl_pn11_A_B_stop, 1000);
	//
	return ccl_ptr->num;
}

/**
  * @brief  сброс циклограммы
  * @param  ccl_ptr: структура управления циклограммой
  * @param  pl_ptr: структура управления ПН-ми
  */
void cyclogram_reset_state(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr)
{
	uint8_t dev_id = ccl_ptr->result.lm_id;
	cyclogram_init(ccl_ptr, pl_ptr, dev_id);
}

/**
  * @brief  инициализация шага циклограммы
  * @param  ccl_ptr: структура управления циклограммой
  * @param  pl_ptr:  структура управления ПН-ми
  * @param  cyclogramm:  номер циклограммы
  * @param  (*function)(type_PL*):  функция, выполняемая на step-шаге
  * @param  delay:  задержка в мс до следующего шага
  */
void cyclogram_step_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogramm, int8_t (*function)(type_CYCLOGRAM_RESULT*, type_PL*), uint32_t delay)
{
	if(ccl_ptr->array[cyclogramm].full_step_num >= STEP_NUM){
		return;
	}
	ccl_ptr->array[cyclogramm].step[ccl_ptr->array[cyclogramm].full_step_num].function = function;
	ccl_ptr->array[cyclogramm].step[ccl_ptr->array[cyclogramm].full_step_num].state = 0;
	ccl_ptr->array[cyclogramm].step[ccl_ptr->array[cyclogramm].full_step_num].delay_to_next_step_ms = delay;
	ccl_ptr->array[cyclogramm].full_step_num += 1;
}

/**
  * @brief  инициализация шага остановки циклограммы при ошибке
  * @param  ccl_ptr: структура управления циклограммой
  * @param  pl_ptr:  структура управления ПН-ми
  * @param  cyclogramm:  номер циклограммы
  * @param  (*function)(type_PL*):  функция, выполняемая на step-шаге
  * @param  delay:  задержка в мс до следующего шага (в данном случае до следующей циклограммы)
  */
void cyclogram_stop_step_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogramm, int8_t (*function)(type_CYCLOGRAM_RESULT*, type_PL*), uint32_t delay)
{
	ccl_ptr->array[cyclogramm].stop_step.function = function;
	ccl_ptr->array[cyclogramm].stop_step.state = 0;
	ccl_ptr->array[cyclogramm].stop_step.delay_to_next_step_ms = delay;
}

/**
  * @brief  запуск экстренной остановки работы циклограммы
  * @param  ccl_ptr: структура управления циклограммой
  * @param  pl_ptr:  структура управления ПН-ми
  */
void cyclogram_stop_step_run(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr)
{
	if ((ccl_ptr->num != 0) || (ccl_ptr->array[ccl_ptr->num].step_timeout != 0) || (ccl_ptr->array[ccl_ptr->num].step_num != 0)){
		#ifdef DEBUG
			printf_time();	printf("-Cyclogram stop\n");
		#endif
		ccl_ptr->array[ccl_ptr->num].step[ccl_ptr->array[ccl_ptr->num].step_num].state = 0; // обнуляем состояние шага циклограммы для корректной обработки в следующий проход
		ccl_ptr->array[ccl_ptr->num].stop_step.function(&ccl_ptr->result, pl_ptr);
		ccl_ptr->array[ccl_ptr->num].step_num = STEP_NUM - 1;
		ccl_ptr->array[ccl_ptr->num].step_timeout = CYCLOGRAM_EMERGENCY_STOP_TIMEOUT;
		ccl_ptr->array[ccl_ptr->num].step[ccl_ptr->array[ccl_ptr->num].step_num].state = 1;
	}
	else{
		//
	}
}

/**
  * @brief  старт работы с циклограммами
  * @param  ccl_ptr: структура управления циклограммой
  * @param  mode:  режим циклограммы: 0-отключить циклограмму, 1-разовый проход циклограммы, 2-циклический режим
  * @param  cyclogram_num:  номер циклограммы
  * @retval статус ошибки: >0 - ок, меньше нуля - не ок
  */
int8_t cyclogram_start(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t mode, uint8_t cyclogram_num)
{
	// при необходимости экстренно останавливаем работу предыдущей циклограммы
	cyclogram_stop_step_run(ccl_ptr, pl_ptr);
	// проверка на корректность режима
	switch(mode){
		case CYCLOGRAM_MODE_OFF:
		case CYCLOGRAM_MODE_SINGLE:
		case CYCLOGRAM_MODE_CYCLIC:
			ccl_ptr->mode = mode;
			break;
		default:
			ccl_ptr->mode = 0;
	}
	//
	ccl_ptr->time_ms = 0;
	//
	cyclogram_single_init(ccl_ptr, pl_ptr, cyclogram_num);
	if ((mode == CYCLOGRAM_MODE_CYCLIC) || (mode == CYCLOGRAM_MODE_SINGLE)){
		#ifdef DEBUG
			printf("\n"); printf_time(); printf("-Cyclogram start %d\n", ccl_ptr->num);
		#endif
	}
	//
	ccl_ptr->state &= ~CYCLOGRAM_STATUS_CCLNUM;
	ccl_ptr->state = ((ccl_ptr->mode << 0) & CYCLOGRAM_STATUS_CCLNUM);
	//
	return 0;
}

/**
  * @brief  инициализация отдельной циклограммы перед началом работы
  * @param  ccl_ptr: структура управления циклограммой
  * @param  cyclogram_num:  номер циклограммы
  */
void cyclogram_single_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogram_num)
{
	uint8_t i;
	if(ccl_ptr->mode == CYCLOGRAM_MODE_CYCLIC){
		ccl_ptr->num = ((cyclogram_num >= 1) && (cyclogram_num < CYCLEGRAMM_NUM)) ? cyclogram_num : 1;
	}
	else{  // проверка на правильный номер циклограммы
		if (cyclogram_num < CYCLEGRAMM_NUM){
			ccl_ptr->num = cyclogram_num;
		}
		else{
			ccl_ptr->num = 1;
		}
	}
	// проверка на корректность номера циклограммы
	ccl_ptr->array[ccl_ptr->num].step_timeout = 0;
	ccl_ptr->array[ccl_ptr->num].state = 0;
	ccl_ptr->array[ccl_ptr->num].step_num = 0;
	for (i=0; i<STEP_NUM; i++){
		ccl_ptr->array[ccl_ptr->num].step[i].state = 0;
	}
}

/**
  * @brief  обработка состояния циклограммы один раз в 100мс
  * @param  pl_ptr: структура управления циклограммой и ПН-ми
	* @param  stop_flag: при установке в 1 циклограмма заканчивается и останавливается
	* @param  period_ms: период вызова данной функции
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t cyclogram_process(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t stop_flag, uint16_t period_ms)
{
	uint8_t c_num, s_num;
	int8_t fanction_report;
	if (ccl_ptr->mode != CYCLOGRAM_MODE_OFF){
		//
		c_num = ccl_ptr->num;
		s_num = ccl_ptr->array[c_num].step_num;
		//
		ccl_ptr->state &= ~CYCLOGRAM_STATUS_CCLNUM;
		ccl_ptr->state = ((ccl_ptr->mode << 0) & CYCLOGRAM_STATUS_CCLNUM);
		//
		ccl_ptr->time_ms += period_ms;
		//
		if(((ccl_ptr->array[c_num].step[s_num].function != 0) || (ccl_ptr->array[c_num].step_timeout)) && (ccl_ptr->array[c_num].step_num < STEP_NUM)){
			if (ccl_ptr->array[c_num].step[s_num].state == 0){ //шаг циклограммы не запущен
				ccl_ptr->array[c_num].step[s_num].state = 1;
				ccl_ptr->array[c_num].step_timeout = ccl_ptr->array[c_num].step[s_num].delay_to_next_step_ms;
				// передаем параметры выполняемой циклограммы в результаты
				ccl_ptr->result.cyclogram_num = c_num;
				ccl_ptr->result.time_ms =	ccl_ptr->time_ms;
				//
				if (ccl_ptr->array[c_num].step[s_num].function != 0){
					fanction_report = ccl_ptr->array[c_num].step[s_num].function(&ccl_ptr->result, pl_ptr);
					#ifdef DEBUG
						printf_time();
						printf("---step_num %d: report %d\n", ccl_ptr->array[c_num].step_num, fanction_report);
					#endif
					if (fanction_report){ // экстренное прекращение работы циклограммы
						#ifdef DEBUG
							printf_time();
							printf("--- ! ALARM ! \n");
						#endif
						cyclogram_stop_step_run(ccl_ptr, pl_ptr);
					}
				}
			} 
			else{
				if(ccl_ptr->array[c_num].step_timeout <= 0){
					ccl_ptr->array[c_num].step[s_num].state = 0;
					ccl_ptr->array[c_num].step_num += 1;
					ccl_ptr->array[c_num].step_timeout = 0;
				}
				else{
					ccl_ptr->array[c_num].step_timeout -= period_ms;
				}
			}
		}
		else{
			if (stop_flag){
				//
				ccl_ptr->state &= ~CYCLOGRAM_STATUS_FULL_ISS_MEM;
				ccl_ptr->state |= (1 << 7) & CYCLOGRAM_STATUS_FULL_ISS_MEM;
			}
			else{
				#ifdef DEBUG
					printf_time();
					printf("-Cyclogram finish %d\n\n", ccl_ptr->num);
				#endif
				ccl_ptr->array[c_num].step_num = 0;
				if (ccl_ptr->mode == CYCLOGRAM_MODE_SINGLE){
					ccl_ptr->mode = 0;
					ccl_ptr->array[c_num].step_timeout = 0;
				}		
				else if (ccl_ptr->mode == CYCLOGRAM_MODE_CYCLIC){
					ccl_ptr->num += 1;
					if (ccl_ptr->num >= CYCLEGRAMM_NUM){
						ccl_ptr->num = 1;
						ccl_ptr->array[c_num].step_timeout = 0;
					}
				}
			}
		}
		return 1;
	}
	else{
		ccl_ptr->num = 0;
		ccl_ptr->array[c_num].step_num = 0;
		ccl_ptr->array[c_num].step_timeout = 0;
		return 0;
	}
}

///*** Общие атомараные функции  ***///
/**
  * @brief инициализация для контэйнера результата циклограммы
  */
int8_t result_init(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	type_PL_CYCLOGRAMA_RESULT *rs_box = &result_ptr->box;
	// обнуляем все параметры работы с результатотом
	result_ptr->cyclogram_result_ready_flag = 0;
	result_ptr->tmi_slice_num = 0;
	result_ptr->body_num = 0;
	result_ptr->body_offset = 0;
	result_ptr->result_num += 1;
	result_ptr->box.header.cyclograma_status = 0;
	// обнуляем все параметры работы с результатотом
	memset((uint8_t*)rs_box, 0xFE, sizeof(type_PL_CYCLOGRAMA_RESULT));
	result_refresh(result_ptr, pl_ptr);
	return 0;
}

/**
  * @brief запись очередных данных, полученных от ПН (для вызова внутри атомарных функций)
  */
int8_t result_write_tmi_slice(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr, uint8_t *tmi_slice)
{
	type_PL_CYCLOGRAMA_RESULT *rs_box = &result_ptr->box;
	memcpy(&rs_box->header.tmi_slice[result_ptr->tmi_slice_num], tmi_slice, sizeof(rs_box->header.tmi_slice[result_ptr->tmi_slice_num]));
	if (result_ptr->tmi_slice_num < (sizeof(rs_box->header.tmi_slice) - 1)){
		result_ptr->tmi_slice_num ++;
	}
	else if (result_ptr->tmi_slice_num < 8){
		//
	}
	//
	result_refresh(result_ptr, pl_ptr);
	//
	return 0;
}

/**
  * @brief запись очередных данных, полученных от ПН
  */
int8_t result_row_data_write(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr, uint8_t *pl_data, uint16_t pl_data_leng)
{
	uint16_t pl_data_remain, pl_data_offset; //pl_data_offset - указатель на первый незаписанный элемент внутри pl_data
	uint8_t body_free_space = 0, body_data_len, wr_data_len=0;
	type_PL_CYCLOGRAMA_RESULT *rs_box = &result_ptr->box;
	//
	body_data_len = sizeof(result_ptr->box.body[0].data);
	//
	body_free_space = body_data_len - result_ptr->body_offset;
	pl_data_remain = pl_data_leng;
	pl_data_offset = 0;
	// #ifdef DEBUG
	// 	printf("-----Write result data: b_num=%d, b_offs=%d, pl_d_remain=%d\n", result_ptr->body_num, result_ptr->body_offset, pl_data_remain);
	// #endif
	//
	while(pl_data_remain != 0){
		//
		body_free_space = body_data_len - result_ptr->body_offset;
		//
		if (body_free_space == 0){
			result_ptr->body_num += 1;
			result_ptr->body_offset = 0;
			body_free_space = body_data_len - result_ptr->body_offset;
			result_refresh(result_ptr, pl_ptr);
		}
		wr_data_len = min(pl_data_remain, body_free_space);
		//
		memcpy((uint8_t*)&rs_box->body[result_ptr->body_num].data[result_ptr->body_offset], pl_data+pl_data_offset, wr_data_len);
		pl_data_remain -= wr_data_len;
		result_ptr->body_offset += wr_data_len;
		pl_data_offset += wr_data_len;
		//
		// #ifdef DEBUG
		// 	printf("-----Write result data: b_num=%d, b_offs=%d, pl_d_remain=%d\n", result_ptr->body_num, result_ptr->body_offset, pl_data_remain);
		// #endif
	}
	//
	result_refresh(result_ptr, pl_ptr);
	//
	return 0;
}

/**
  * @brief обновляем все поля после добавления данных
  */
int8_t result_refresh(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	type_PL_CYCLOGRAMA_RESULT *rs_box = &result_ptr->box;
	uint8_t body_num;
	// создаем заголовок для заголовка 
	body_num = (result_ptr->body_offset > 0) ? (result_ptr->body_num + 1) : result_ptr->body_num;
	frame_create_header((uint8_t*)&rs_box->header.header, result_ptr->lm_id, ARCH_HEADER_FRAME_TYPE, DATA_TYPE_CYCLOGRAM_RESULT, result_ptr->result_num, body_num);
	// поправляем переменные для заголовка
	rs_box->header.result_num = result_ptr->result_num;
	rs_box->header.cyclograma_mode = result_ptr->cyclogram_num;
	// обновляем crc для заголовка
	frame_crc16_calc((uint8_t*)&rs_box->header);
	// обнавляем заголовок для старого тела и создаем для нового
	// обновляем crc для старого тела и создаем для нового
	if (body_num > 0){
		frame_create_header((uint8_t*)&rs_box->body[body_num-1].header, result_ptr->lm_id, ARCH_BODY_FRAME_TYPE, DATA_TYPE_CYCLOGRAM_RESULT, result_ptr->result_num, body_num-1);
		frame_crc16_calc((uint8_t*)&rs_box->body[body_num-1]);
		if (body_num > 1){
			frame_create_header((uint8_t*)&rs_box->body[body_num-2].header, result_ptr->lm_id, ARCH_BODY_FRAME_TYPE, DATA_TYPE_CYCLOGRAM_RESULT, result_ptr->result_num, body_num-2);
			frame_crc16_calc((uint8_t*)&rs_box->body[body_num-2]);
		}
	}
	return 0;
	//
}

/**
  * @brief финишируем удачную запись данных и передаем сигнал наружу, по которому данные перекладываются на CAN и в Память
  */
int8_t result_finish(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	result_ptr->box.header.cyclograma_status |= 0x00;
	result_refresh(result_ptr, pl_ptr);
	result_ptr->cyclogram_result_ready_flag = 1;
	return 0;
}

/**
  * @brief финишируем неудачную циклограммузапись данных и передаем сигнал наружу, по которому данные перекладываются на CAN и в Память
  */
int8_t result_emergency_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	result_ptr->box.header.cyclograma_status |= 0x01;
	result_refresh(result_ptr, pl_ptr);
	result_ptr->cyclogram_result_ready_flag = 1;
	return 0;
}

///*** ПН1.1_А - атомараные функции  ***///

int8_t pl_pn11A_init(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_reset_state(&pl_ptr->_11A);
	#ifdef DEBUG
		printf_time(); printf("--PL11A reset\n");
	#endif
	return 0;
}

int8_t pl_pn11A_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11A, PN11_OUTPUT_DEFAULT);
	#ifdef DEBUG
		printf_time(); printf("--PL11A output set default\n");
	#endif
	return 0;
}

int8_t pl_pn11A_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	int8_t retval=0, tmi_check = 0;
	type_PN11_TMI_slice tmi_slice;
	tmi_check = pn_11_tmi_slice_get_and_check(&pl_ptr->_11A, (uint8_t*)&tmi_slice);
	if (tmi_check){
		retval = 1; 
	}
	else{
		retval = 0;
	}
	result_write_tmi_slice(result_ptr, pl_ptr, (uint8_t*)&tmi_slice);
	#ifdef DEBUG
		printf_time();
		printf("--PL11A tmi slice %d: tmi_num %d, pl_type %d, U %.1f, I %.1f, temp %d\n", 
																																									retval,
																																									tmi_slice.number,
																																									tmi_slice.pl_type,
																																									tmi_slice.voltage/16.,
																																									tmi_slice.current/16.,
																																									tmi_slice.temp
																																									);
	#endif
	return retval;
}

int8_t pl_pn11A_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_on(&pl_ptr->_11A);
	#ifdef DEBUG
		printf_time();
		printf("--PL11A pwr_on\n");
	#endif
	return 0;
}

int8_t pl_pn11A_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_off(&pl_ptr->_11A);
	#ifdef DEBUG
		printf_time();
		printf("--PL11A pwr off\n");
	#endif
	return 0;
}

int8_t pl_pn11A_fpga_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11A, PN11_OUTPUT_FPGA_ON);
	//debug
	printf_time();
	printf("--PL11A fpga on\n");
	//
	return 0;
}

int8_t pl_pn11A_interface_reset_and_sync(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	tr_lvl_reset(&pl_ptr->_11A.interface.tr_lvl);
	pn_11_interface_synch(&pl_ptr->_11A);
	//debug
	printf_time();
	printf("--PL11A reset and sync interface\n");
	//
	return 0;
}

int8_t pl_pn11A_write_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	uint32_t mode = 0x00000001;
	pn_11_write_u32_data(&pl_ptr->_11A, PN_11_MEM_ADDR_MODE, &mode, 1);
	
	//debug
	printf_time();
	printf("--PL11A write mode: %d\n", mode);
	//
	return 0;
}

int8_t pl_pn11A_read_req_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	//делаем запрос на чтение режима
	pn_11_seq_read_start(&pl_ptr->_11A, PN_11_MEM_ADDR_MODE, 1);
	//debug
	printf_time();
	printf("--PL11A mode read request\n");
	//
	return 0;
}

int8_t pl_pn11A_read_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	uint32_t mode;
	mode = pl_ptr->_11A.mem.field.mode;
	//debug
	printf_time();
	printf("--PL11A mode read: %d\n", mode);
	//
	return 0;
}

int8_t pl_pn11A_read_req_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	//делаем запрос на чтение режима
	pn_11_seq_read_start(&pl_ptr->_11A, PN_11_MEM_ADDR_START_MEM, sizeof(type_PN_11_MEM)/4);
	#ifdef DEBUG
		printf_time(); printf("--PL11A read all request\n");
	#endif
	return 0;
}

int8_t pl_pn11A_read_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	result_row_data_write(result_ptr, pl_ptr, (uint8_t*)pl_ptr->_11A.mem.array.data, 4*184);
	#ifdef DEBUG
		printf_time();
		printf("--PL11A read all:\n\t\tmode-%08X ptm_1-%08X ptm_2-%08X, ptm_3-%08X, error-%04X\n", 
					pl_ptr->_11A.mem.field.mode,
					pl_ptr->_11A.mem.field.ptm[0],
					pl_ptr->_11A.mem.field.ptm[1],
					pl_ptr->_11A.mem.field.ptm[2],
					pl_ptr->_11A.error_flags
					);
	#endif
	return 0;
}

int8_t pl_pn11A_fpga_mcu_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11A, PN11_OUTPUT_FPGA_MCU_ON);
	//debug
	printf_time();
	printf("--PL11A mcu on\n");
	//
	return 0;
}

int8_t pl_pn11A_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_off(&pl_ptr->_11A);
	result_emergency_stop(result_ptr, pl_ptr);
	#ifdef DEBUG
		printf_time(); printf("--PL11A stop\n");
	#endif
	return 0;
}

///*** ПН1.1_Б - атомараные функции  ***///

int8_t pl_pn11B_init(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_reset_state(&pl_ptr->_11B);
	#ifdef DEBUG
		printf_time();
		printf("--PL11B reset\n");
	#endif
	return 0;
}

int8_t pl_pn11B_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11B, PN11_OUTPUT_DEFAULT);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL11B output set default\n");
	#endif
	//
	return 0;
}

int8_t pl_pn11B_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	int8_t retval=0, tmi_check = 0;
	type_PN11_TMI_slice tmi_slice;
	tmi_check = pn_11_tmi_slice_get_and_check(&pl_ptr->_11B, (uint8_t*)&tmi_slice);
	if (tmi_check){
		retval = 1; 
	}
	else{
		retval = 0;
	}
	result_write_tmi_slice(result_ptr, pl_ptr, (uint8_t*)&tmi_slice);
	#ifdef DEBUG
		printf_time();
		printf("--PL11B tmi slice %d: tmi_num %d, pl_type %d, U %.1f, I %.1f, temp %d\n", 
																																										retval,
																																										tmi_slice.number,
																																										tmi_slice.pl_type,
																																										tmi_slice.voltage/16.,
																																										tmi_slice.current/16.,
																																										tmi_slice.temp
																																										);
	#endif
	return retval;
}

int8_t pl_pn11B_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_on(&pl_ptr->_11B);
	//debug
	printf_time();
	printf("--PL11B pwr_on\n");
	//
	return 0;
}

int8_t pl_pn11B_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_off(&pl_ptr->_11B);
	//debug
	printf_time();
	printf("--PL11B pwr off\n");
	//
	return 0;
}

int8_t pl_pn11B_fpga_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11B, PN11_OUTPUT_FPGA_ON);
	//debug
	printf_time();
	printf("--PL11B fpga on\n");
	//
	return 0;
}

int8_t pl_pn11B_interface_reset_and_sync(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	tr_lvl_reset(&pl_ptr->_11B.interface.tr_lvl);
	pn_11_interface_synch(&pl_ptr->_11B);
	//debug
	printf_time();
	printf("--PL11B reset and sync interface\n");
	//
	return 0;
}

int8_t pl_pn11B_write_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	uint32_t mode = 0x00000002;
	pn_11_write_u32_data(&pl_ptr->_11B, PN_11_MEM_ADDR_MODE, &mode, 1);
	
	//debug
	printf_time();
	printf("--PL11B write mode: %d\n", mode);
	//
	return 0;
}

int8_t pl_pn11B_read_req_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	//делаем запрос на чтение режима
	pn_11_seq_read_start(&pl_ptr->_11B, PN_11_MEM_ADDR_MODE, 1);
	//debug
	printf_time();
	printf("--PL11B mode read request\n");
	//
	return 0;
}

int8_t pl_pn11B_read_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	uint32_t mode;
	mode = pl_ptr->_11B.mem.field.mode;
	//debug
	printf_time();
	printf("--PL11B mode read: %d\n", mode);
	//
	return 0;
}

int8_t pl_pn11B_read_req_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	//делаем запрос на чтение режима
	pn_11_seq_read_start(&pl_ptr->_11B, PN_11_MEM_ADDR_START_MEM, sizeof(type_PN_11_MEM)/4);
	//debug
	printf_time();
	printf("--PL11B read all request\n");
	//
	return 0;
}

int8_t pl_pn11B_read_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	result_row_data_write(result_ptr, pl_ptr, (uint8_t*)pl_ptr->_11B.mem.array.data, 4*184);
	#ifdef DEBUG
		printf_time();
		printf("--PL11B read all:\n\t\tmode-%08X ptm_1-%08X ptm_2-%08X, ptm_3-%08X, error-%04X\n", 
					pl_ptr->_11B.mem.field.mode,
					pl_ptr->_11B.mem.field.ptm[0],
					pl_ptr->_11B.mem.field.ptm[1],
					pl_ptr->_11B.mem.field.ptm[2],
					pl_ptr->_11B.error_flags
					);
	#endif
	//
	return 0;
}

int8_t pl_pn11B_fpga_mcu_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_output_set(&pl_ptr->_11B, PN11_OUTPUT_FPGA_MCU_ON);
	#ifdef DEBUG
		printf_time(); printf("--PL11B mcu on\n");
	#endif
	return 0;
}

int8_t pl_pn11B_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_off(&pl_ptr->_11B);
	result_emergency_stop(result_ptr, pl_ptr);
	#ifdef DEBUG
		printf_time(); printf("--PL11B stop\n");
	#endif
	//
	return 0;
}

///*** ПН1.1_А ПН1.1_Б - атомараные функции  ***///

int8_t pl_pn11_A_B_write_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	uint32_t mode_a, mode_b;
	mode_a = 0x00000005;
	pn_11_write_u32_data(&pl_ptr->_11A, PN_11_MEM_ADDR_MODE, &mode_a, 1);
	mode_b = 0x0000000B;
	pn_11_write_u32_data(&pl_ptr->_11B, PN_11_MEM_ADDR_MODE, &mode_b, 1);
	#ifdef DEBUG
		printf_time(); printf("--PL11A и PL11B write mode_a:%d, mode_b:%d\n", mode_a, mode_b);
	#endif
	return 0;
}

int8_t pl_pn11_A_B_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_11_pwr_off(&pl_ptr->_11A);
	pn_11_pwr_off(&pl_ptr->_11B);
	result_emergency_stop(result_ptr, pl_ptr);
	#ifdef DEBUG
		printf_time(); printf("--PL11A & PL11B stop\n");
	#endif
	return 0;
}

///*** ПН1.2 - атомараные функции  ***///
int8_t pl_pn12_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_pwr_on(&pl_ptr->_12);
	//debug
	printf_time();
	printf("--PL12 pwr_on\n");
	//
	return 0;
}

int8_t pl_pn12_interface_init(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_interface_init(&pl_ptr->_12, pl_ptr->_12.interface.tr_lvl.huart);
	//debug
	printf_time();
	printf("--PL12 pwr_on\n");
	//
	return 0;
}

int8_t pl_pn12_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_pwr_off(&pl_ptr->_12);
	//debug
	printf_time();
	printf("--PL12 pwr_off\n");
	//
	return 0;
}

int8_t pl_pn12_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	int8_t retval=0, tmi_check = 0;
	type_PN11_TMI_slice tmi_slice;
	tmi_check = pn_12_tmi_slice_get_and_check(&pl_ptr->_12, (uint8_t*)&tmi_slice);
	if (tmi_check){
		retval = 1; 
	}
	else{
		retval = 0;
	}
	result_write_tmi_slice(result_ptr, pl_ptr, (uint8_t*)&tmi_slice);
	#ifdef DEBUG
		printf_time();
		printf("--PL12 tmi slice %d: tmi_num %d, pl_type %d, U %.1f, I %.1f, temp %d\n", 
																																									retval,
																																									tmi_slice.number,
																																									tmi_slice.pl_type,
																																									tmi_slice.voltage/16.,
																																									tmi_slice.current/16.,
																																									tmi_slice.temp
																																									);
	#endif
	return retval;
}

int8_t pl_pn12_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_output_set(&pl_ptr->_12, PN12_OUTPUT_DEFAULT);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL12 output set default\n");
	#endif
	//
	return 0;
}

int8_t pl_pn12_set_iku_test(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_output_set(&pl_ptr->_12, 0x03);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL12 output set default\n");
	#endif
	//
	return 0;
}

int8_t pl_pn12_set_iku_spi_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_output_set(&pl_ptr->_12, PN12_OUTPUT_FPGA_SPI_SET);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL12 output SPI on\n");
	#endif
	//
	return 0;
}

int8_t pl_pn12_set_iku_spi_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_output_set(&pl_ptr->_12, PN12_OUTPUT_FPGA_SPI_RESET);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL12 output SPI off\n");
	#endif
	//
	return 0;
}

int8_t pl_pn12_read_req_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	//делаем запрос на чтение
	pn_12_seq_read_start(&pl_ptr->_12, PN_12_MEM_ADDR_START_MEM, sizeof(type_PN_12_MEM)/4);
	#ifdef DEBUG
		printf_time(); printf("--PL12 read all request\n");
	#endif
	return 0;
}

int8_t pl_pn12_read_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	result_row_data_write(result_ptr, pl_ptr, (uint8_t*)pl_ptr->_12.mem.data, sizeof(type_PN_12_MEM));
	#ifdef DEBUG
		printf_time();
		printf("--PL12 read all:\n\t\tword_1-%08X size-%d, error-%04X\n", 
					pl_ptr->_12.mem.data[0],
					sizeof(type_PN_12_MEM),
					pl_ptr->_12.error_flags
					);
	#endif
	return 0;
}

int8_t pl_pn12_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_12_output_set(&pl_ptr->_12, PN12_OUTPUT_DEFAULT);
	pn_12_pwr_off(&pl_ptr->_12);
	result_emergency_stop(result_ptr, pl_ptr);
	//debug
	printf_time();
	printf("--PL12 stop\n");
	//
	return 0;
}

///*** ПН2.0 - атомараные функции  ***///
int8_t pl_pn20_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_pwr_on(&pl_ptr->_20);
	//debug
	printf_time();
	printf("--PL20 pwr_on\n");
	//
	return 0;
}

int8_t pl_pn20_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_pwr_off(&pl_ptr->_20);
	//debug
	printf_time();
	printf("--PL20 pwr_off\n");
	//
	return 0;
}

int8_t pl_pn20_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	int8_t retval=0, tmi_check = 0;
	type_PN20_TMI_slice tmi_slice;
	tmi_check = pn_20_tmi_slice_get_and_check(&pl_ptr->_20, (uint8_t*)&tmi_slice);
	if (tmi_check){
		retval = 1; 
	}
	else{
		retval = 0;
	}
	result_write_tmi_slice(result_ptr, pl_ptr, (uint8_t*)&tmi_slice);
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tmi slice %d: tmi_num %d, pl_type %d, U %.1f, I %.1f, temp %d\n", 
																																									retval,
																																									tmi_slice.number,
																																									tmi_slice.pl_type,
																																									tmi_slice.voltage/16.,
																																									tmi_slice.current/16.,
																																									tmi_slice.temp
																																									);
	#endif
	return retval;
}

int8_t pl_pn20_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_output_set(&pl_ptr->_20, PN20_OUTPUT_DEFAULT);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 output set default\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_set_iku_pl_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_output_set(&pl_ptr->_20, PN20_PL_ON);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 output set RST On\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_read_sw_tmi_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sw_tmi_comm, PN_20_SW_TEST_U16_LENG, pl_ptr->_20.mem.sw_tmi_start, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sw_tmi start\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_read_sw_tmi_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sw_tmi_comm, PN_20_SW_TEST_U16_LENG, pl_ptr->_20.mem.sw_tmi_stop, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sw_tmi stop\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_tas_start_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.tas_start_comm, PN_20_TAS_START_U16_LENG, pl_ptr->_20.mem.tas_start_1, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tas_start 1\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_tas_read_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.tas_result_comm, PN_20_TAS_RESULT_U16_LENG, pl_ptr->_20.mem.tas_result_1, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tas_read 1\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_tas_start_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.tas_start_comm, PN_20_TAS_START_U16_LENG, pl_ptr->_20.mem.tas_start_2, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tas_start 2\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_tas_read_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.tas_result_comm, PN_20_TAS_RESULT_U16_LENG, pl_ptr->_20.mem.tas_result_2, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tas_read 2\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_tas_start_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.tas_start_comm, PN_20_TAS_START_U16_LENG, pl_ptr->_20.mem.tas_start_3, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tas_start 3\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_tas_read_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.tas_result_comm, PN_20_TAS_RESULT_U16_LENG, pl_ptr->_20.mem.tas_result_3, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 tas_read 3\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_sd_start_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sd_start_comm, PN_20_SD_START_U16_LENG, pl_ptr->_20.mem.sd_start_1, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sd_start 1\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_sd_read_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sd_result_comm, PN_20_SD_RESULT_U16_LENG, pl_ptr->_20.mem.sd_result_1, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sd_read 1\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_sd_start_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sd_start_comm, PN_20_SD_START_U16_LENG, pl_ptr->_20.mem.sd_start_2, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sd_start 2\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_sd_read_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sd_result_comm, PN_20_SD_RESULT_U16_LENG, pl_ptr->_20.mem.sd_result_2, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sd_read 2\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_sd_start_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sd_start_comm, PN_20_SD_START_U16_LENG, pl_ptr->_20.mem.sd_start_3, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sd_start 3\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_sd_read_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_comm_queue_start(&pl_ptr->_20, pl_ptr->_20.interface.sd_result_comm, PN_20_SD_RESULT_U16_LENG, pl_ptr->_20.mem.sd_result_3, 300);
	//
	#ifdef DEBUG
		printf_time();
		printf("--PL20 sd_read 3\n");
	#endif
	//
	return 0;
}

int8_t pl_pn20_write_result(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	result_row_data_write(result_ptr, pl_ptr, (uint8_t*)&pl_ptr->_20.mem, sizeof(type_PN20_Mem));
	#ifdef DEBUG
		printf_time();
		printf("--PL20 write result\n");
	#endif
	return 0;
}

int8_t pl_pn20_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr)
{
	pn_20_output_set(&pl_ptr->_20, PN20_OUTPUT_DEFAULT);
	pn_20_pwr_off(&pl_ptr->_20);
	result_emergency_stop(result_ptr, pl_ptr);
	//debug
	printf_time();
	printf("--PL20 stop\n");
	//
	return 0;
}
