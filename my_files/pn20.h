#ifndef _PN20_H
#define _PN20_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "tmp1075.h"
#include "pn_11_interface_app_lvl.h"
#include "debug.h"

#include "rtc.h"

#define PN20_OUTPUT_DEFAULT 0x0F
#define PN20_OUTPUT_FPGA_ON 0x02
#define PN20_OUTPUT_FPGA_MCU_ON 0x00
//
#define PN29_TEMP_MAX (85<<8)
#define PN20_TEMP_MIN (-30<<8)

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
#define PN_20_VOLT_MAX 		5.5
#define PN_20_VOLT_MIN 		4.5
// границы мощности Вт
#define PN_20_PWR_MAX 		15.0
#define PN_20_PWR_MIN 		1.0

// задержка для определения ошибки питания - устанавливается каждый ра, когда происходит изменение состояния питания
#define PN_20_PWR_TIMEOUT_MS 	1000

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
} type_PN20_report; 			//18

#pragma pack(8)

/** 
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_GPIO_setting input[4]; // 0-C_TM_PWR_ERR, 1-C_TM_CPU_OK, 2-C_TM_INT, 3-C_TM_ERR
	type_GPIO_setting output[4]; // 0-C_TK_nRESET, 1-C_TK_SPI_SEL, 2-KU_2, 3-KU_3
	type_PWR_CHANNEL* pwr_ch;
	type_TMP1075_DEVICE* tmp_ch;
	uint16_t interrupt_timeout;
	type_PN11_INTERFACE_APP_LVL interface;
	type_PN20_report report;
	uint16_t status, error_flags;
	uint8_t error_cnt;
} type_PN20_model;

void pn_20_init(type_PN20_model* pn20_ptr, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart);
void pn_20_reset_state(type_PN20_model* pn20_ptr);
void pn_20_output_set(type_PN20_model* pn20_ptr, uint8_t output_state);
void pn_20_report_create(type_PN20_model* pn20_ptr);
uint8_t pn_20_get_inputs_state(type_PN20_model* pn20_ptr);
uint8_t pn_20_get_outputs_state(type_PN20_model* pn20_ptr);
void pn_20_pwr_on(type_PN20_model* pn20_ptr);
void pn_20_pwr_off(type_PN20_model* pn20_ptr);

void pn_20_dbg_reset_state(type_PN20_model* pn20_ptr);


#endif
