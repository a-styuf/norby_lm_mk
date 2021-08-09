#ifndef _PN20_H
#define _PN20_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "tmp1075.h"
#include "pn_11_interface_app_lvl.h"
#include "debug.h"

#include "rtc.h"

#define PN20_OUTPUT_DEFAULT 		0x00
#define PN20_PL_ON 							0x01

//Статусы работы
#define PN20_STATUS_WORK		 		(0x01<<0)
#define PN20_STATUS_ERROR		 		(0x01<<1)
#define PN20_STATUS_INH			 		(0x0F<<4)

//Ошибки работы
#define PN20_NO_ERROR			 			(0)
#define PN20_STM1_ERROR 				(1<<0)
#define PN20_STM2_ERROR 				(1<<1)
#define PN20_STM3_ERROR	 				(1<<2)
#define PN20_STM4_ERROR		 			(1<<3)
#define PN20_ERR_PWR						(1<<4) //длина 4 бита
#define PN20_TEMP_ERROR		 			(1<<8) //длина 4 бита
#define PN20_INT_ERR_BAD_FRAME	(1<<12)
#define PN20_INT_ERR_TIMEOUT		(1<<13)
#define PN20_INT_ERR_OTHER			(1<<14)
#define PN20_OTHER_ERROR 				(1<<15)

// пороговые значения для определения ошибки температуры: в 1/256°C
#define PN_20_TEMP_HIGH 				(80*256)
#define PN_20_TEMP_LOW 					(-30*256)
#define PN_20_TEMP_HYST					(1*256)

// пороговые значения для определения ошибки питания: напряжение в В, ток в А, мощность в Вт
// пороги напряжения В	
#define PN_20_VOLT_MAX 					5.5
#define PN_20_VOLT_MIN 					4.5
// границы мощности Вт
#define PN_20_PWR_MAX 					1.0
#define PN_20_PWR_MIN 					0.5

// задержка для определения ошибки питания - устанавливается каждый раз, когда происходит изменение состояния питания
#define PN_20_PWR_PERIODICAL_TIMEOUT_MS 	1000
#define PN_20_PWR_ON_OFF_TIMEOUT_MS 			5000

// задержка для для проверок температуры, что бы не долбить модуль температуры очень часто
#define PN_20_TMP_PERIODICAL_TIMEOUT_MS 	1000

// типы запретов работы ПН
#define PN_20_INH_SELF 											(1 << 0)
#define PN_20_INH_PWR												(1 << 1)
#define PN_20_INH_TMP												(1 << 2)

// дефайны для переменных
#define PN_20_UART_TIMEOUT_MS								250

#define PN_20_UART_RX_STATUS_OK							1
#define PN_20_UART_RX_STATUS_SHORT					0
#define PN_20_UART_RX_STATUS_BAD_EOT_SOF		-1
#define PN_20_UART_RX_STATUS_BAD_CRC				-2

#define PN_20_SW_TEST_U16_LENG							10
#define PN_20_TAS_START_U16_LENG						1
#define PN_20_TAS_RESULT_U16_LENG						5
#define PN_20_SD_START_U16_LENG							1
#define PN_20_SD_RESULT_U16_LENG						5


#pragma pack(2)
/** 
  * @brief  структура хранения отчета по ПН1.1 (18 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint16_t status; 				//+0
	uint16_t error_flags; 	//+2
	uint8_t err_cnt;	 			//+4
	uint8_t inh;			 			//+5
	uint16_t voltage; 			//+6
	uint16_t current; 			//+8
	uint16_t temp;		 			//+10
	uint8_t outputs; 				//+12
	uint8_t inputs; 				//+13
	uint16_t rsrv[2];		 			//+14
} type_PN20_report; 			//18

/**
  * @brief  struct to store telemetry data slice
  */
typedef struct{
  uint8_t number;       //+0
  uint8_t pl_type;      //+1
  uint8_t voltage;      //+2
  uint8_t current;      //+3
  uint8_t outputs;      //+4
  uint8_t inputs;       //+5
  uint8_t temp;         //+6
  uint8_t pl_error_cnt; //+7
  uint16_t pl_errors;   //+8
  uint16_t pl_status;   //+10
} type_PN20_TMI_slice;	//12  //SLICE - срез

/** 
  * @brief  структура формирования параметров для хранения в ПЗУ ПН (18 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint8_t inhibit; 				//+0
	uint8_t gap; 						//+1
	uint8_t rsrv[16]; 			//+2
} type_PN20_сfg;		 			//18

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
} type_PN20_frame; //6

/** 
  * @brief  результаты чтения данных ПН20
  */
typedef struct
{
	uint16_t sw_tmi_start[PN_20_SW_TEST_U16_LENG];
	uint16_t tas_start_1[PN_20_TAS_START_U16_LENG];
	uint16_t tas_result_1[PN_20_TAS_RESULT_U16_LENG];
	uint16_t tas_start_2[PN_20_TAS_START_U16_LENG];
	uint16_t tas_result_2[PN_20_TAS_RESULT_U16_LENG];
	uint16_t tas_start_3[PN_20_TAS_START_U16_LENG];
	uint16_t tas_result_3[PN_20_TAS_RESULT_U16_LENG];
	uint16_t sd_start_1[PN_20_SD_START_U16_LENG];
	uint16_t sd_result_1[PN_20_SD_RESULT_U16_LENG];
	uint16_t sd_start_2[PN_20_SD_START_U16_LENG];
	uint16_t sd_result_2[PN_20_SD_RESULT_U16_LENG];
	uint16_t sd_start_3[PN_20_SD_START_U16_LENG];
	uint16_t sd_result_3[PN_20_SD_RESULT_U16_LENG];
	uint16_t sw_tmi_stop[PN_20_SW_TEST_U16_LENG];	
} type_PN20_Mem; //6

/** 
  * @brief  структура хранения параметров информационного интерфейса
  */
typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t tx_data[256], tx_len, tx_cnt;
	uint8_t rx_data[256], rx_len, rx_cnt, rx_ptr, rx_ptr_offset, row_rx_data[128], row_rx_len;
	uint16_t rx_timeout;
	uint8_t rx_timeout_flag, rx_data_ready;
	type_PN20_frame rx_msg, tx_msg;
	//управление очередью чтения
	uint8_t sw_tmi_comm[PN_20_SW_TEST_U16_LENG];
	uint8_t tas_start_comm[PN_20_TAS_START_U16_LENG];
	uint8_t tas_result_comm[PN_20_TAS_RESULT_U16_LENG];
	uint8_t sd_start_comm[PN_20_SD_START_U16_LENG];
	uint8_t sd_result_comm[PN_20_SD_RESULT_U16_LENG];
	uint8_t comm_queue_array[64], comm_queue_cnt, comm_queue_flg, comm_queue_leng;
	uint16_t queue_interval_ms, queue_step_time_ms;
	uint16_t *array_to_save;
	uint16_t array_to_save_trash;
	uint8_t array_to_save_index;
	//
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
	//
	type_PWR_CHANNEL* pwr_ch;
	type_TMP1075_DEVICE* tmp_ch;
	uint16_t pwr_check_timeout_ms, tmp_check_timeout_ms;
	//
	uint8_t	inhibit; 	//флаги для запрета: 0 - включения ПН, 1 - отключения по питанию, 2 - отключению по температуре.
	//
	type_PN20_interface interface;
	type_PN20_Mem mem;
	//
	type_PN20_report report;
	type_PN20_TMI_slice tmi_slice;
	uint8_t tmi_slice_number;
	//
	type_PN20_сfg loaded_cfg;
	type_PN20_сfg cfg;
	//
	uint8_t self_num; // собственный номе ПН
	uint16_t status, error_flags;
	uint8_t error_cnt;
} type_PN20_model;

void pn_20_init(type_PN20_model* pn20_ptr, uint8_t num, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart);
void pn_20_reset_state(type_PN20_model* pn20_ptr);
void pn_20_process(type_PN20_model* pn20_ptr, uint16_t period_ms);

void pn_20_tmi_slice_create(type_PN20_model* pn20_ptr);
int8_t pn_20_tmi_slice_get_and_check(type_PN20_model* pn20_ptr, uint8_t *slice);
void pn_20_report_create(type_PN20_model* pn20_ptr);
void pn_20_report_reset(type_PN20_model* pn20_ptr);
void pn_20_set_inh(type_PN20_model* pn20_ptr, uint8_t inh);
uint8_t pn_20_get_short_status(type_PN20_model* pn20_ptr);

void pn_20_output_set(type_PN20_model* pn20_ptr, uint8_t output_state);
uint8_t pn_20_get_inputs_state(type_PN20_model* pn20_ptr);
uint8_t pn_20_get_outputs_state(type_PN20_model* pn20_ptr);

void pn_20_pwr_process(type_PN20_model* pn20_ptr, uint16_t period_ms);
uint8_t pn_20_pwr_check(type_PN20_model* pn20_ptr);
void pn_20_pwr_on(type_PN20_model* pn20_ptr);
void pn_20_pwr_off(type_PN20_model* pn20_ptr);

void pn_20_tmp_process(type_PN20_model* pn20_ptr, uint16_t period_ms);
uint8_t pn_20_tmp_check(type_PN20_model* pn20_ptr);

void pn_20_int_init(type_PN20_model* pn20_ptr, UART_HandleTypeDef *uart_ptr);
void pn_20_interface_process(type_PN20_model* pn20_ptr, uint16_t period_ms);
void pn_20_int_rx_ptr_reset(type_PN20_model* pn20_ptr);
void pn_20_int_send_frame(type_PN20_model* pn20_ptr, uint8_t data_len, uint8_t data_hb, uint8_t data_lb);
uint8_t pn_20_instasend(type_PN20_model* pn20_ptr, uint8_t* insta_send_data);
void pn_20_send_start_tas_test(type_PN20_model* pn20_ptr);
void pn_20_send_start_sd_test(type_PN20_model* pn20_ptr);
void pn_20_comm_queue_start(type_PN20_model* pn20_ptr, uint8_t* comm_queue, uint8_t comm_queue_leng, uint16_t* mem_to_save, uint16_t interval_ms);
void pn_20_comm_queue_process(type_PN20_model* pn20_ptr, uint16_t pause_ms);
void pn_20_int_rx_huart_cb(type_PN20_model* pn20_ptr);
void pn_20_int_rx_timeout_cb(type_PN20_model* pn20_ptr, uint16_t period_ms);
int8_t pn_20_int_check_frame(type_PN20_model* pn20_ptr, uint8_t* data, uint8_t len);
int8_t pn_20_int_get_last_data(type_PN20_model* pn20_ptr, uint8_t* data);
void pn_20_int_tx_prcs_cb(type_PN20_model* pn20_ptr);
void pn_20_int_err_prcs_cb(type_PN20_model* pn20_ptr);

uint8_t pn_20_get_cfg(type_PN20_model* pn20_ptr, uint8_t *cfg);
uint8_t pn_20_set_cfg(type_PN20_model* pn20_ptr, uint8_t *cfg);

void  _pn_20_error_collector(type_PN20_model* pn12_ptr, uint16_t error, int16_t data);

void pn_20_dbg_test(type_PN20_model* pn20_ptr);

#endif
