#ifndef _PL_CYCLOGRAM_H
#define _PL_CYCLOGRAM_H

#include "pn11.h"
#include "pn12.h"
#include "pn20.h"
#include "pn_dcr.h"
#include "lm_interfaces.h"
#include <stdio.h>
#include "debug.h"

#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

// настройка циклограмм
#define LM	 		(0)
#define PL11A	 	(1)
#define PL11B	 	(2)
#define PL12		(3)
#define PL20 		(4)
#define PL_DCR1 (5)
#define PL_DCR2 (6)

// раскрашивание переменных
#define CYCLEGRAMM_NUM 										(16)
#define STEP_NUM 													(64)
#define CYCLOGRAM_EMERGENCY_STOP_TIMEOUT 	(60000)

//режимы работы
#define CYCLOGRAM_MODE_OFF 			(0)
#define CYCLOGRAM_MODE_SINGLE 	(1)
#define CYCLOGRAM_MODE_CYCLIC 	(2)

//ошибки циклограмм
#define CYCLOGRAM_STATUS_OK						(0x00<<0)
#define CYCLOGRAM_STATUS_CCLNUM				(0x7F<<0)
#define CYCLOGRAM_STATUS_FULL_ISS_MEM	(0x01<<7)

typedef struct
{ 
	type_PN11_model _11A;
	type_PN11_model _11B;
	type_PN12_model _12;
	type_PN20_model _20;
	type_PN_DCR_model _dcr;
} type_PL;

// CYCLOGRAM //

typedef struct
{ 
	type_PL_CYCLOGRAMA_RESULT box;
	uint8_t cyclogram_result_ready_flag;
	uint8_t lm_id;
	uint16_t result_num;
	uint8_t tmi_slice_num;
	uint8_t cyclogram_num;
	uint8_t body_num, body_offset; //body_num - количество используемых дополнительных частей контейнера, body_offset - количество записанных данных в отдельный body 
	uint32_t time_ms;
} type_CYCLOGRAM_RESULT;

typedef struct
{ 
	int8_t (*function)(type_CYCLOGRAM_RESULT*, type_PL*);
	uint32_t state;
	uint32_t delay_to_next_step_ms;
} type_CYCLOGRAM_single_step;

typedef struct
{ 
	type_CYCLOGRAM_single_step step[STEP_NUM];
	type_CYCLOGRAM_single_step stop_step;
	uint32_t state;
	int32_t step_timeout;
	uint8_t step_num;
	uint8_t full_step_num;
} type_CYCLOGRAM_control;

// PL //
typedef struct
{ 
	type_CYCLOGRAM_control array[CYCLEGRAMM_NUM];
	//
	type_CYCLOGRAM_RESULT result;
	//
	uint32_t time_ms;
	uint8_t num;
	uint8_t mode;
	uint8_t state;  // 6-0: значение режима циклограммы, 7 - "1" ожидается очистка памяти, "0" нормальная работа
} type_CYCLOGRAM;

void pl_init(type_PL* pl_ptr, type_PWR_CHANNEL* pwr_arr, type_TMP1075_DEVICE* tmp_arr, UART_HandleTypeDef* huart11A, UART_HandleTypeDef* huart11B, UART_HandleTypeDef* huart12, UART_HandleTypeDef* huart20, UART_HandleTypeDef* huartDCR);
void pl_report_get(type_PL* pl_ptr, uint8_t pl_num, uint8_t* report, uint8_t* len);

int8_t cyclogram_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t dev_id);
void cyclogram_reset_state(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr);
void cyclogram_step_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogramm, int8_t (*function)(type_CYCLOGRAM_RESULT*, type_PL*), uint32_t delay);
void cyclogram_stop_step_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogramm, int8_t (*function)(type_CYCLOGRAM_RESULT*, type_PL*), uint32_t delay);
void cyclogram_stop_step_run(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr);
int8_t cyclogram_start(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t mode, uint8_t cyclogram_num);
void cyclogram_single_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogram_num);
int8_t cyclogram_process(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t stop_flag, uint16_t period_ms);

int8_t result_init(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t result_write_tmi_slice(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr, uint8_t *tmi_slice);
int8_t result_row_data_write(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr, uint8_t *pl_data, uint16_t pl_data_leng);
int8_t result_refresh(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t result_finish(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t result_emergency_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);

// pl_1.1A
int8_t pl_pn11A_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_interface_reset_and_sync(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_fpga_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_fpga_mcu_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_write_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_read_req_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_read_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_read_req_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_read_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11A_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);

// pl_1.1B
int8_t pl_pn11B_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_interface_reset_and_sync(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_fpga_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_fpga_mcu_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_write_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_read_req_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_read_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_read_req_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_read_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11B_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);

// pl1.1А & B
int8_t pl_pn11_A_B_write_mode(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn11_A_B_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);

// pl1.2
int8_t pl_pn12_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_interface_init(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_set_iku_test(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_set_iku_spi_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_set_iku_spi_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_read_req_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_read_all(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn12_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);

// pl2.0
int8_t pl_pn20_pwr_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_pwr_off(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_check_and_save_tmi(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_set_iku_default(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_set_iku_pl_on(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_read_sw_tmi_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_read_sw_tmi_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_tas_start_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_tas_read_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_tas_start_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_tas_read_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_tas_start_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_tas_read_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_sd_start_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_sd_read_1(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_sd_start_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_sd_read_2(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_sd_start_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_sd_read_3(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_write_result(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
int8_t pl_pn20_stop(type_CYCLOGRAM_RESULT* result_ptr, type_PL* pl_ptr);
#endif
