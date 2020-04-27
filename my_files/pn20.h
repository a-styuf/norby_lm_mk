#ifndef _PN20_H
#define _PN20_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "tmp1075.h"
#include "pn_11_interface_app_lvl.h"
#include "debug.h"

#include "rtc.h"

#define PN20_OUTPUT_DEFAULT 0x00
//
#define PN20_TEMP_MAX (60<<8)
#define PN20_TEMP_MIN (-50<<8)

//Ошибки работы
#define PN_20_ERR_NO_ERROR 					(0x00)
#define PN_20_ERR_SYS								(0x01 << 1)
#define PN_20_ERR_TEMPERATURE				(0x01 << 2)
#define PN_DCR_ERR_OTHER						(0x01 << 3)
#define PN_DCR_ERR_PWR							(0x01 << 4)
#define PN_DCR_ERR_INTERFACE				(0x01 << 12)

//Ошибки UART
#define PN_20_INTERFACE_ERR_NO_ERROR 			(0x00)
#define PN_20_INTERFACE_ERR_BAD_FRAME			(0x01 << 0)
#define PN_20_INTERFACE_ERR_TIMEOUT				(0x01 << 1)
#define PN_20_INTERFACE_ERR_HAL						(0x01 << 2)
#define PN_20_INTERFACE_ERR_OTHER					(0x01 << 3)

// пороговые значения для определения ошибки питания: напряжение в В, ток в А, мощность в Вт
// пороги напряжения В	
#define PN_20_VOLT_MAX 		5.5
#define PN_20_VOLT_MIN 		4.5
// границы мощности Вт
#define PN_20_PWR_MAX 		15.0
#define PN_20_PWR_MIN 		1.0

// задержка для определения ошибки питания - устанавливается каждый ра, когда происходит изменение состояния питания
#define PN_20_PWR_TIMEOUT_MS 	1000

// таймаут ответа
#define PN_20_UART_TIMEOUT_MS 	100


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


/** 
  * @brief  объеденение двух структур для простоты использования
  */
typedef union
{
	struct {
		uint8_t sof;
		uint8_t len;
		uint8_t data_hb;
		uint8_t data_lb;
		uint8_t crc8;
		uint8_t eot;
	} frame;
	struct {
		uint8_t frame[6];
	} array;
} type_PN20_frame;

/** 
  * @brief  структура хранения параметров информационного интерфейса
  */
typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t tx_data[256], tx_len, tx_cnt;
	uint8_t rx_data[256], rx_buff[256], rx_len, rx_cnt, rx_ptr;
	uint16_t rx_timeout;
	type_PN20_frame rx_msg, tx_msg;
	uint8_t error_flags;
	uint8_t error_cnt;
} type_PN20_interface;


#pragma pack(8)

/** 
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_GPIO_setting input[4];  // 0-SYS_FAILURE, 1-CURRENT_MON, 2-INTERRUPT, 3-TM_ANA
	type_GPIO_setting output[4];  // 0-EXT_RESET, 1..3-KU_1..3
	type_PWR_CHANNEL* pwr_ch;
	type_TMP1075_DEVICE* tmp_ch;
	type_PN20_interface interface;
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

void pn_20_int_init(type_PN20_interface *int_ptr, UART_HandleTypeDef *uart_ptr);
void pn_20_int_send(type_PN20_interface *int_ptr, uint8_t* data, uint8_t len);
void pn_20_int_send_frame(type_PN20_interface *int_ptr, uint8_t data_len, uint8_t data_hb, uint8_t data_lb);
void pn_20_int_rx_huart_cb(type_PN20_interface *int_ptr);
void pn_20_int_rx_timeout_cb(type_PN20_interface *int_ptr, uint16_t period_ms);
void pn_20_int_check_frame(type_PN20_interface *int_ptr, uint8_t data, uint8_t len);
void pn_20_int_tx_prcs_cb(type_PN20_interface *int_ptr);
void pn_20_int_err_prcs_cb(type_PN20_interface *int_ptr);
void  _pn_20_int_error_collector(type_PN20_interface *int_ptr, uint16_t error);


void pn_20_dbg_test(type_PN20_model* pn20_ptr);

#endif
