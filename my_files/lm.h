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
#include "debug.h"

#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

typedef unsigned short uint16_t;

// настройки прибора
#define DEV_ID (0x06)
#define SOFT_VERSION "0.14"
// свойства микроконтроллера

// раскрашивание переменных

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
	uint16_t error_flags; // +0
	uint8_t err_cnt; // +8
	uint8_t rst_cnt; // +7
	uint16_t voltage; // +2
	uint16_t current; // +4
	uint16_t temperature; // +6
} type_LM_REPORT;

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
	type_CYCLOGRAM cyclogram;
	type_PL pl;
	type_LM_INTERFACES interface;
	type_LM_REPORT report;
} type_LM_DEVICE;

// прототипы функций

//*** ITB ***//
void lm_init(type_LM_DEVICE* lm_ptr);
int8_t lm_ctrl_init(type_LM_DEVICE* lm_ptr);
void lm_report_create(type_LM_DEVICE* lm_ptr);

int8_t pwr_init(type_PWR_CONTROL* pwr_ptr, I2C_HandleTypeDef* hi2c_ptr);
void pwr_on_off(type_PWR_CONTROL* pwr_ptr, uint8_t pwr_switches);
void pwr_process_100ms(type_PWR_CONTROL* pwr_ptr);
void pwr_create_report(type_PWR_CONTROL* pwr_ptr);
void pwr_alert_gd_it_process(type_PWR_CONTROL* pwr_ptr, uint16_t it_position);
void pwr_cb_it_process(type_PWR_CONTROL* pwr_ptr, uint8_t error);

int8_t tmp_init(type_TMP_CONTROL* tmp_ptr, I2C_HandleTypeDef* hi2c_ptr);
void tmp_process_100ms(type_TMP_CONTROL* tmp_ptr);
void tmp_alert_it_process(type_TMP_CONTROL* tmp_ptr, uint16_t it_position);
void tmp_cb_it_process(type_TMP_CONTROL* tmp_ptr, uint8_t error);

void fill_tmi_and_beacon(type_LM_DEVICE* lm_ptr);
void fill_gen_tmi(type_LM_DEVICE* lm_ptr);
void fill_dcr_rx_frame(type_LM_DEVICE* lm_ptr);

uint16_t com_ans_form(uint8_t req_id, uint8_t self_id, uint8_t* seq_num, uint8_t type, uint8_t leng, uint8_t* com_data, uint8_t* ans_com);
void printf_time(void);

#endif
