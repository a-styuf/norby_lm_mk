#ifndef __LM_H
#define __LM_H

#include <stdlib.h>
#include <stdio.h>
#include "my_gpio.h"
#include "i2c.h"
#include "rtc.h"
#include "crc16.h"
#include "math.h"
#include "tmp1075.h"
#include "pn11.h"
#include "pn12.h"
#include "pn20.h"
#include "pwr_ch.h"
#include "pl_cyclogram.h"
#include "usart.h"
#include "lm_interfaces.h"
#include "ext_mem.h"
#include "clock.h"
#include "debug.h"


#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))


typedef unsigned short uint16_t;

// настройки прибора
#define DEV_ID (0x06)
#define SOFT_VERSION "1.07.0"
// свойства микроконтроллера


// раскрашивание переменных

// настройка оставноки\запуска циклограмм по заполнению памяти
#define ISS_MEM_TOP_BOUND_PROCENTAGE 		95 //граница отключения работы циклограмм
#define ISS_MEM_BOT_BOUND_PROCENTAGE 		90 //граница включения работы циклограмм


// биты статуса
#define LM_STATUS_INH							(0x0F << 4)
#define LM_STATUS_CCLGRM					(0xFF << 8)

// типы запретов работы ПН
#define LM_INH_SELF 						(1 << 0)
#define LM_INH_PWR							(1 << 1)
#define LM_INH_TMP							(1 << 2)
#define LM_INH_CCL_MEM_CHECK		(1 << 3)

// описание рабочих структур
// Power //
typedef struct
{
	uint32_t gpio_state;  // 31:pwr_gd, 30:pwr_alert, 29-21:NU, 20-18: ena[2:0] LM ... 2-0: ena[2:0] PL_DCR2
	uint16_t lm_voltage, lm_current;
	uint16_t pl11a_voltage, pl11a_current;
	uint16_t pl11b_voltage, pl11b_current;
	uint16_t pl12_voltage, pl12_current;
	uint16_t pl20_voltage, pl20_current;
	uint16_t pl_dcr1_voltage, pl_dcr1_current;
	uint16_t pl_dcr2_voltage, pl_dcr2_current;
} type_PWR_REPORT;

typedef struct
{ 
	type_PWR_CHANNEL ch[7]; // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
	type_GPIO_setting gd, alert;
	uint8_t ch_read_queue;
	type_PWR_REPORT report;
} type_PWR_CONTROL;

// Temperature //
typedef struct
{
	uint8_t gpio_state;  // 0: tmp_alert
	uint16_t temperature[5]; // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0
} type_TMP_REPORT;

typedef struct
{ 
	type_TMP1075_DEVICE tmp1075[5]; // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0
	type_GPIO_setting alert;
	uint8_t ch_read_queue;
	type_TMP_REPORT report;
} type_TMP_CONTROL;

/**
  * @brief  структура для хранения переменных управления командами (продолжительными со статусами, IdVar = 0х02)
  */
typedef struct
{ 
	uint32_t start_time_s, time_ms, point_time_ms;
	uint32_t main_counter;
	uint8_t ena;
} type_CMD_CONTROL;

/**
  * @brief  отчет по работе LM (18-байт, для помещение в кадр общей телеметрии)
  */
typedef struct
{ 
	uint16_t status; // +0
	uint16_t error_flags; // +2
	uint8_t err_cnt; // +3
	uint8_t rst_cnt; // +4
	uint16_t voltage; // +6
	uint16_t current; // +8
	uint16_t temperature; // +10
	uint16_t iss_mem_wr_ptr; // +12
	uint16_t dcr_mem_wd_ptr; // +14
	uint16_t rsrv; // +16
} type_LM_REPORT; // +18

/** 
  * @brief  структура формирования параметров для хранения в ПЗУ ПН (26 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint32_t iss_wr_ptr; 				//+0
	uint32_t iss_rd_ptr; 				//+4
	uint32_t dcr_wr_ptr; 				//+8
	uint32_t dcr_rd_ptr; 				//+12
	uint8_t inhibit; 						//+16
	uint8_t cyclogram_mode; 		//+17
	uint8_t cyclogram_num; 			//+18
	uint8_t rst_cnt;						//+19
	uint16_t result_num;				//+20
	uint8_t rsrv[4];						//+22
} type_LM_сfg;		 						//26

/**
  * @brief  управляющие параметры МС
  */
typedef struct
{ 
	uint32_t global_time_s;
	uint16_t status;
	uint16_t error_flags;
	uint8_t err_cnt;
	uint8_t rst_cnt;
	uint16_t pl_status;
	uint16_t constant_mode;  //1 - режим констант включен, 0 - режим констант отключен
	//
	uint8_t inhibit;  // 0-2 - NU, 3 - запрет проверки оставшегося свободного места для работы циклограмм ИСС
} type_LM_CTRL;

// lm //
/** 
  * @brief  структура хранения всех настроек платы
  */
typedef struct
{
	type_LM_CTRL ctrl;
	type_MEM_CONTROL mem;
	type_PWR_CONTROL pwr;
	type_TMP_CONTROL tmp;
	type_CMD_CONTROL cmd_ctrl[CMD_POOL_LEN];  // параметры для управления долгими командами
	//
	type_CYCLOGRAM cyclogram;
	type_PL pl;
	uint8_t pl_cyclogram_stop_flag; //переменная, отвечающая за разрешение работы циклограмм ИСС
	//
	type_LM_INTERFACES interface;
	type_LM_REPORT report;
	type_LM_сfg loaded_cfg, cfg;
	type_LM_CFG_Frame loaded_cfg_frame, cfg_frame_to_save;
} type_LM_DEVICE;

// прототипы функций

//*** ITB ***//
void lm_init(type_LM_DEVICE* lm_ptr);
int8_t lm_ctrl_init(type_LM_DEVICE* lm_ptr);
void lm_report_create(type_LM_DEVICE* lm_ptr);
int8_t lm_load_parameters(type_LM_DEVICE* lm_ptr);
int8_t lm_save_parameters(type_LM_DEVICE* lm_ptr);
void lm_get_cfg(type_LM_DEVICE* lm_ptr, uint8_t *cfg);
uint8_t lm_set_cfg(type_LM_DEVICE* lm_ptr, uint8_t *cfg);
void lm_pl_inhibit_set(type_LM_DEVICE* lm_ptr, uint8_t pl_num, uint8_t inh);
void lm_set_inh(type_LM_DEVICE* lm_ptr, uint8_t inh);
void lm_cyclogram_process(type_LM_DEVICE* lm_ptr, uint16_t period_ms);
uint16_t lm_get_pl_status(type_LM_DEVICE* lm_ptr);
void lm_reset_state(type_LM_DEVICE* lm_ptr);

int8_t pwr_init(type_PWR_CONTROL* pwr_ptr, I2C_HandleTypeDef* hi2c_ptr);
void pwr_on_off(type_PWR_CONTROL* pwr_ptr, uint8_t pwr_switches);
void pwr_process(type_PWR_CONTROL* pwr_ptr, uint16_t period_ms);
void pwr_create_report(type_PWR_CONTROL* pwr_ptr);
void pwr_alert_gd_it_process(type_PWR_CONTROL* pwr_ptr, uint16_t it_position);
void pwr_cb_it_process(type_PWR_CONTROL* pwr_ptr, uint8_t error);

int8_t tmp_init(type_TMP_CONTROL* tmp_ptr, I2C_HandleTypeDef* hi2c_ptr);
void tmp_process(type_TMP_CONTROL* tmp_ptr, uint16_t period_ms);
void tmp_alert_it_process(type_TMP_CONTROL* tmp_ptr, uint16_t it_position);
void tmp_cb_it_process(type_TMP_CONTROL* tmp_ptr, uint8_t error);

void fill_tmi_and_beacon(type_LM_DEVICE* lm_ptr);
void fill_gen_tmi(type_LM_DEVICE* lm_ptr);
void fill_dcr_rx_frame(type_LM_DEVICE* lm_ptr);
void fill_pl_iss_last_frame(type_LM_DEVICE* lm_ptr);
void fill_pl_cyclogramm_result(type_LM_DEVICE* lm_ptr);

uint16_t com_ans_form(uint8_t req_id, uint8_t self_id, uint8_t* seq_num, uint8_t type, uint8_t leng, uint8_t* com_data, uint8_t* ans_com);
uint32_t get_uint32_val_from_bound(uint32_t val, uint32_t min, uint32_t max); //если число внутри границ - используется оно, если нет, то ближайшая граница
void pl_iss_get_app_lvl_reprot(uint8_t pl_type, uint8_t *instasend_data, char *report_ctr);

#endif
