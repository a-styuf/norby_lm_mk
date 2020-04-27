#ifndef _PN11_H
#define _PN11_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "tmp1075.h"
#include "pn_11_interface_app_lvl.h"
#include "debug.h"

#include "rtc.h"

#define PN11_OUTPUT_DEFAULT 0x0F
#define PN11_OUTPUT_FPGA_ON 0x02
#define PN11_OUTPUT_FPGA_MCU_ON 0x00
//
#define PN11_TEMP_MAX (85<<8)
#define PN11_TEMP_MIN (-30<<8)

#define PN11_CURR_MAX (2<<8)
#define PN11_CURR_MIN (1<<0)
#define PN11_VOLT_MAX (6<<8)
#define PN11_VOLT_MIN (4<<8)
#define PN11_PWR_MAX (10<<8)
#define PN11_PWR_MIN (1<<0)

//Ошибки работы
#define STATE_NO_ERROR			 	(0)
#define STATE_TEMP_ERROR		 	(1<<0)
#define STATE_CURRENT_ERROR  	(1<<1)
#define STATE_VOLTAGE_ERROR		(1<<2)
#define STATE_INTERFACE_ERROR	(1<<3)
#define STATE_INT_ERROR 			(1<<4)
#define STATE_CPU_ERROR 			(1<<5)
#define STATE_FPGA_ERROR 			(1<<5)

// пороговые значения для определения ошибки питания: напряжение в В, ток в А, мощность в Вт

// пороги напряжения В	
#define PN_11_VOLT_MAX 		5.5
#define PN_11_VOLT_MIN 		4.5
// границы мощности Вт
#define PN_11_PWR_MAX 		12.0
#define PN_11_PWR_MIN 		1.0

// задержка для определения ошибки питания - устанавливается каждый ра, когда происходит изменение состояния питания
#define PN_11_PWR_TIMEOUT_MS 	1000

#pragma pack(2)
/** 
  * @brief  структура хранения отчета по ПН1.1 (18 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint16_t status; 				//+0
	uint16_t error_flags; 	//+2
	uint8_t err_cnt;	 			//+4
	uint8_t gap;			 			//+5
	uint16_t voltage; 			//+6
	uint16_t current; 			//+8
	uint16_t temp;		 			//+10
	uint8_t outputs; 				//+12
	uint8_t inputs; 				//+13
	uint16_t rsrv[2];		 			//+14
} type_PN11_report; 			//18

#pragma pack(8)

/** 
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_GPIO_setting input[4]; // 0-INT, 1-PWR_ERROR, 2-WATCHDOG, 3-CPU_ERROR
	type_GPIO_setting output[4]; // 0-RST_FPGA, 1-RST_LEON, 2-KU_2, 3-KU_3
	type_PWR_CHANNEL* pwr_ch;
	type_TMP1075_DEVICE* tmp_ch;
	uint16_t interrupt_timeout;
	type_PN11_INTERFACE_APP_LVL interface;
	type_PN11_report report;
	uint16_t status, error_flags;
	uint8_t error_cnt;
} type_PN11_model;

void pn_11_init(type_PN11_model* pn11_ptr, uint8_t num, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart);
void pn_11_reset_state(type_PN11_model* pn11_ptr);
void pn_11_output_set(type_PN11_model* pn11_ptr, uint8_t output_state);
void pn_11_report_create(type_PN11_model* pn11_ptr);
uint8_t pn_11_get_inputs_state(type_PN11_model* pn11_ptr);
uint8_t pn_11_get_outputs_state(type_PN11_model* pn11_ptr);
void pn_11_pwr_on(type_PN11_model* pn11_ptr);
void pn_11_pwr_off(type_PN11_model* pn11_ptr);

void pn_11_dbg_test(type_PN11_model* pn11_ptr);


#endif
