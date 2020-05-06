#ifndef _PL_CYCLOGRAM_H
#define _PL_CYCLOGRAM_H

#include "pn11.h"
#include "pn12.h"
#include "pn20.h"
#include "pn_dcr.h"
#include <stdio.h>
#include "debug.h"

#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

// настройка циклограмм
#define PL11A	 	(1)
#define PL11B	 	(2)
#define PL12		(3)
#define PL20 		(4)
#define PL_DCR1 (5)
#define PL_DCR2 (6)

// раскрашивание переменных
#define CYCLEGRAMM_NUM 	(16)
#define STEP_NUM 				(32)

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
	int8_t (*function)(type_PL*);
	uint32_t state;
	uint32_t delay_to_next_step_ms;
} type_CYCLOGRAM_single_step;

typedef struct
{ 
	type_CYCLOGRAM_single_step step[32];
	uint32_t state;
	int32_t step_timeout;
	uint8_t step_num;
	uint8_t full_step_num;
} type_CYCLOGRAM_control;

// PL //
typedef struct
{ 
	type_CYCLOGRAM_control array[16];
	uint32_t time_ms;
	uint8_t num;
	uint8_t mode;
} type_CYCLOGRAM;

void pl_init(type_PL* pl_ptr, type_PWR_CHANNEL* pwr_arr, type_TMP1075_DEVICE* tmp_arr, UART_HandleTypeDef* huart11A, UART_HandleTypeDef* huart11B, UART_HandleTypeDef* huart12, UART_HandleTypeDef* huart20, UART_HandleTypeDef* huartDCR);
void pl_report_get(type_PL* pl_ptr, uint8_t pl_num, uint8_t* report, uint8_t* len);

void cyclogram_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr);
void cyclogram_step_init(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr, uint8_t cyclogramm, int8_t (*function)(type_PL*), uint32_t delay);
int8_t cyclogram_start(type_CYCLOGRAM* ccl_ptr, uint8_t mode, uint8_t cyclogram_num);
void cyclogram_single_init(type_CYCLOGRAM* ccl_ptr, uint8_t cyclogram_num);
int8_t cyclogram_process_100ms(type_CYCLOGRAM* ccl_ptr, type_PL* pl_ptr);

// pl_1.1A
int8_t pl_pn11A_set_iku_default(type_PL* pl_ptr);
int8_t pl_pn11A_check_temp(type_PL* pl_ptr);
int8_t pl_pn11A_pwr_on(type_PL* pl_ptr);
int8_t pl_pn11A_pwr_check(type_PL* pl_ptr);
int8_t pl_pn11A_interface_sync(type_PL* pl_ptr);
int8_t pl_pn11A_interface_reset(type_PL* pl_ptr);
int8_t pl_pn11A_fpga_on(type_PL* pl_ptr);
int8_t pl_pn11A_pwr_off(type_PL* pl_ptr);
int8_t pl_pn11A_fpga_mcu_on(type_PL* pl_ptr);
int8_t pl_pn11A_get_and_check_hw_telemetry(type_PL* pl_ptr);
int8_t pl_pn11A_check_INT(type_PL* pl_ptr);
int8_t pl_pn11A_write_mode(type_PL* pl_ptr);
int8_t pl_pn11A_read_req_mode(type_PL* pl_ptr);
int8_t pl_pn11A_read_mode(type_PL* pl_ptr);
#endif
