#ifndef _PN11_H
#define _PN11_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "tmp1075.h"
#include "pn_11_interface_app_lvl.h"
#include "debug.h"

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

//Статусы работы
#define PN11_STATUS_WORK		 		(0x01<<0)
#define PN11_STATUS_ERROR		 		(0x01<<1)
#define PN11_STATUS_INH			 		(0x0F<<4)

//Ошибки работы
#define PN11_NO_ERROR			 		(0)
#define PN11_CPU_ERROR 				(1<<0) // NU
#define PN11_FPGA_ERROR 			(1<<1) // NU
#define PN11_INT_ERROR	 			(1<<2) // NU
#define PN11_NU_ERROR		 			(1<<3) // NU
#define PN11_ERR_PWR					(1<<4) //длина 4 бита
#define PN11_TEMP_ERROR		 		(1<<8) //длина 4 бита
#define PN11_INTERFACE_ERROR	(1<<12)
#define PN11_APP_LVL_ERROR		(1<<13)
#define PN11_TR_LVL_ERROR			(1<<14)
#define PN11_OTHER_ERROR 			(1<<15)

// пороговые значения для определения ошибки температуры: в 1/256°C
#define PN_11_TEMP_HIGH 	(80*256)
#define PN_11_TEMP_LOW 		(-30*256)
#define PN_11_TEMP_HYST		(1*256)

// пороговые значения для определения ошибки питания: напряжение в В, ток в А, мощность в Вт
// пороги напряжения В	
#define PN_11_VOLT_MAX 		5.5
#define PN_11_VOLT_MIN 		4.5
// границы мощности Вт
#define PN_11_PWR_MAX 		1.0
#define PN_11_PWR_MIN 		0.5

// задержка для определения ошибки питания - устанавливается каждый раз, когда происходит изменение состояния питания
#define PN_11_PWR_PERIODICAL_TIMEOUT_MS 	1000
#define PN_11_PWR_ON_OFF_TIMEOUT_MS 			5000

// задержка для для проверок температуры, что бы не долбить модуль температуры очень часто
#define PN_11_TMP_PERIODICAL_TIMEOUT_MS 	1000

// шаг обработки чтения памяти ПН
#define PN_11_READ_MEM_TIMEOUT_MS 	100

// объем данных для полного вычитывания
#define PN_11_APP_LVL_ADDR_OFFSET     0xB0000000
#define PN_11_MEM_ADDR_MODE       (PN_11_APP_LVL_ADDR_OFFSET + 0x0)
#define PN_11_MEM_ADDR_START_MEM  (PN_11_APP_LVL_ADDR_OFFSET + 0x0)

// типы запретов работы ПН
#define PN_11_INH_SELF 						(1 << 0)
#define PN_11_INH_PWR							(1 << 1)
#define PN_11_INH_TMP							(1 << 2)

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
} type_PN11_report; 			//18

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
} type_PN11_TMI_slice;	//12  //SLICE - срез

/** 
  * @brief  структура памяти для взаимодействия с ПН_ИСС
  */
typedef union
{
	struct {
		uint32_t mode;
		uint32_t packet_1[60];
		uint32_t packet_2[60];
		uint32_t packet_3[60];
		uint32_t ptm[3];
	} field;
	struct {
		uint32_t data[184];
	} array;
} type_PN_11_MEM;

/** 
  * @brief  структура формирования параметров для хранения в ПЗУ ПН (18 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint8_t inhibit; 				//+0
	uint8_t gap; 						//+1
	uint8_t rsrv[16]; 			//+2
} type_PN11_сfg;		 			//18

#pragma pack(8)

/** 
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_GPIO_setting input[4]; // 0-INT, 1-PWR_ERROR, 2-WATCHDOG, 3-CPU_ERROR
	type_GPIO_setting output[4]; // 0-RST_FPGA, 1-RST_LEON, 2-KU_2, 3-KU_3
	//
	type_PWR_CHANNEL* pwr_ch;
	type_TMP1075_DEVICE* tmp_ch;
	uint16_t pwr_check_timeout_ms, tmp_check_timeout_ms;
	//
	uint8_t	inhibit; 	//флаги для запрета: 0 - включения ПН, 1 - отключения по питанию, 2 - отключению по температуре.
	//
	type_PN11_INTERFACE_APP_LVL interface;
	//
	uint32_t rd_seq_start_addr, rd_seq_stop_addr, rd_seq_leng;
	uint32_t rd_seq_curr_addr, rd_seq_part_leng;
	uint16_t rd_seq_timeout;
	uint8_t rd_seq_mode;  // режим последовательного чтения данных: 0 - нет чтения, 1 - чтение в процессе
	type_PN_11_MEM mem;
	//
	type_PN11_report report;
	type_PN11_TMI_slice tmi_slice;
	uint8_t tmi_slice_number;
	//
	type_PN11_сfg loaded_cfg;
	type_PN11_сfg cfg;
	//
	uint8_t self_num; // собственный номе ПН
	uint16_t status, error_flags;
	uint8_t error_cnt;
} type_PN11_model;

void pn_11_init(type_PN11_model* pn11_ptr, uint8_t num, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr, UART_HandleTypeDef* huart);
void pn_11_reset_state(type_PN11_model* pn11_ptr);
void pn_11_process(type_PN11_model* pn11_ptr, uint16_t period_ms);

void pn_11_tmi_slice_create(type_PN11_model* pn11_ptr);
int8_t pn_11_tmi_slice_get_and_check(type_PN11_model* pn11_ptr, uint8_t *slice);
void pn_11_report_create(type_PN11_model* pn11_ptr);
void pn_11_report_reset(type_PN11_model* pn11_ptr);
void pn_11_set_inh(type_PN11_model* pn11_ptr, uint8_t inh);
uint8_t pn_11_get_short_status(type_PN11_model* pn11_ptr);

void pn_11_output_set(type_PN11_model* pn11_ptr, uint8_t output_state);
uint8_t pn_11_get_inputs_state(type_PN11_model* pn11_ptr);
uint8_t pn_11_get_outputs_state(type_PN11_model* pn11_ptr);

void pn_11_pwr_process(type_PN11_model* pn11_ptr, uint16_t period_ms);
uint8_t pn_11_pwr_check(type_PN11_model* pn11_ptr);
void pn_11_pwr_on(type_PN11_model* pn11_ptr);
void pn_11_pwr_off(type_PN11_model* pn11_ptr);

void pn_11_tmp_process(type_PN11_model* pn11_ptr, uint16_t period_ms);
uint8_t pn_11_tmp_check(type_PN11_model* pn11_ptr);

void pn_11_interface_init(type_PN11_model* pn11_ptr, UART_HandleTypeDef* huart);
void pn_11_interface_reset(type_PN11_model* pn11_ptr);
void pn_11_interface_synch(type_PN11_model* pn11_ptr);
void pn_11_interface_process(type_PN11_model* pn11_ptr, uint16_t period_ms);
uint8_t pn_11_get_last_frame(type_PN11_model* pn11_ptr, uint8_t *data);
uint8_t pn_11_get_last_frame_in_128B_format(type_PN11_model* pn11_ptr, uint8_t *data);
void pn_11_read_req_u32_data(type_PN11_model* pn11_ptr, uint32_t addr, uint8_t u32_len);
void pn_11_write_u32_data(type_PN11_model* pn11_ptr, uint32_t addr, uint32_t *u32_data, uint8_t u32_len);
uint8_t pn_11_can_instasend(type_PN11_model* pn11_ptr, uint8_t* insta_send_data);
void pn_11_seq_read_start(type_PN11_model* pn11_ptr, uint32_t start_addr, uint32_t u32_leng);
void _pn_11_seq_read_request(type_PN11_model* pn11_ptr);

uint8_t pn_11_get_cfg(type_PN11_model* pn11_ptr, uint8_t *cfg);
uint8_t pn_11_set_cfg(type_PN11_model* pn11_ptr, uint8_t *cfg);

void  _pn_11_error_collector(type_PN11_model* pn11_ptr, uint16_t error, int16_t data);

void pn_11_dbg_test(type_PN11_model* pn11_ptr);
void pn_11_dbg_tr_lvl_test(type_PN11_model* pn11_ptr);

#endif
