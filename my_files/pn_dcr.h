#ifndef _PN_DCR_H
#define _PN_DCR_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "usart.h"
#include "ext_mem.h"
#include "clock.h"

#include "rtc.h"

//#define DEBUG_DCR

//
#define DCR_MODE_OFF 								0
#define DCR_MODE_DEFAULT 						1
#define DCR_MODE_FLIGHT_TASK_1 			2
#define DCR_MODE_FLIGHT_TASK_2 			3
#define DCR_MODE_PAUSE			 				4

#define DCR_MODE_SINGLE			 				0
#define DCR_MODE_CYCLE			 				1

#define DCR_PRELODED_FL_TASK_DEFAULT			0x00
#define DCR_PRELODED_FL_TASK_OTHER				0x01
#define DCR_PRELODED_FL_TASK_FROM_CAN			0xFF

// типы команды для полетного задания
#define DCR_PN_FLT_NO_ONE		 							0x00
#define DCR_PN_FLT_TYPE_PWR 							0x01
#define DCR_PN_FLT_TYPE_UART							0x02
#define DCR_PN_FLT_TYPE_PRST_CMD					0x03
#define DCR_PN_FLT_PASS			 							0x08

#define PN_DCR_PWR_MCU			 0x01
#define PN_DCR_PWR_MSR			 0x02
#define PN_DCR_PWR_ALL			 0x03

#define DCR_PN_PRST_CMD_SNC_TIME			 0x00

// статус ДеКоР
#define PN_DCR_STATUS_MODE 						(0x03 << 0)
#define PN_DCR_STATUS_TAG_MODE				(0x01 << 2)
#define PN_DCR_STATUS_ERROR						(0x01 << 3)
#define PN_DCR_STATUS_INH							(0x01 << 4)
#define PN_DCR_CCL_STEP_NUM						(0xFF << 8)

// ошибки работы ДеКоР
#define PN_DCR_ERR_NO_ERROR 					(0x00)
#define PN_DCR_ERR_WRONG_FRAME_LENG 	(0x01 << 0)
#define PN_DCR_ERR_FL_TASK_ERROR			(0x01 << 1)
#define PN_DCR_ERR_WRONG_MODE					(0x01 << 2)
#define PN_DCR_ERR_OTHER							(0x01 << 3)
#define PN_DCR_ERR_PWR_MCU						(0x01 << 4)
#define PN_DCR_ERR_PWR_MSR						(0x01 << 8)
#define PN_DCR_ERR_UART								(0x01 << 12)

#define PN_DCR_UART_ERR_NO_ERROR 			(0x00)
#define PN_DCR_UART_ERR_BAD_FRAME			(0x01 << 0)
#define PN_DCR_UART_ERR_HAL_NOISE			(0x01 << 1)
#define PN_DCR_UART_ERR_HAL_OTHER			(0x01 << 2)
#define PN_DCR_UART_ERR_OTHER					(0x01 << 3)

#define PN_DCR_UART_FRAME_SHORT				(0x01)
#define PN_DCR_UART_FRAME_LONG				(0x02)
#define PN_DCR_UART_FRAME_SYNCH_TIME	(0x03)

#define PN_DCR_CMD_GET_TM_STATUS			(0x01)
#define PN_DCR_CMD_GET_DATA_MONITOR		(0x02)
#define PN_DCR_CMD_GET_DATA_MASSIVE		(0x03)
#define PN_DCR_CMD_SYNCH_TIME					(0x04)

// пороговые значения для определения ошибки питания: напряжение в В, ток в А, мощность в Вт
// пороги напряжения общие для двух каналов
#define PN_DCR_VOLT_MAX 		9.0
#define PN_DCR_VOLT_MIN 		4.5
// границы мощности для линии питания МК
#define PN_DCR_MC_PWR_MAX 	0.4
#define PN_DCR_MC_PWR_MIN 	0.004
// границы мощности для лини питания датчика
#define PN_DCR_MSR_PWR_MAX 	1.6
#define PN_DCR_MSR_PWR_MIN 	0.1
// задержка для определения ошибки питания - устанавливается каждый ра, когда происходит изменение состояния питания
#define PN_DCR_PWR_PERIODICAL_TIMEOUT_MS 	1000
#define PN_DCR_PWR_ON_OFF_TIMEOUT_MS 			5000
// типы запретов работы ПН
#define PN_11_INH_SELF 						(1 << 0)


#pragma pack(2)
/** 
  * @brief  структура хранения отчета по Декор (18 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint16_t status; 				//+0
	uint16_t error_flags; 	//+2
	uint8_t err_cnt;	 			//+4
	uint8_t outputs; 				//+5
	uint16_t voltage_mcu; 	//+6
	uint16_t current_mcu; 	//+8
	uint16_t voltage_msr; 	//+10
	uint16_t current_msr; 	//+12
	uint8_t uart_rx_cnt;		//+14
	uint8_t uart_tx_cnt;		//+15
	uint16_t rsrv;					//+16
} type_PNDCR_report; 			//18

/** 
  * @brief  объеденение двух структур для простоты использования
  */
typedef union
{
	struct {
		uint8_t start_header;
		uint8_t cmd_type;
		uint8_t cmd_code;
		uint8_t data[118];
		uint8_t stop_tail[3];
	} lng;
	struct {
		uint8_t start_header;
		uint8_t cmd_type;
		uint8_t cmd_code;
		uint8_t data[6];
		uint8_t stop_tail[3];
	} shrt;
	struct {
		uint8_t start_header;
		uint8_t cmd_type;
		uint32_t unix_time;
		uint8_t rsrv[3];
		uint8_t stop_tail[3];
	} snc_time;
} type_PN_DCR_frame;

/** 
  * @brief  структура хранения параметров информационного интерфейса
  */
typedef struct
{
	UART_HandleTypeDef* huart;
	uint8_t tx_data[256], tx_len, tx_cnt;
	uint8_t rx_buff[256], rx_data[256], rx_len, rx_cnt, rx_ptr;
	uint8_t error_flags;
	uint8_t error_cnt;
} type_PNDCR_interface;

/** 
  * @brief  один шаг полетного задания (16 байт)
  */
typedef struct
{
	uint8_t type; //+0
	uint8_t cmd; //+1
	uint32_t pause_ms; //+2
	uint16_t repeate_cnt; //+6
	uint8_t data[8]; //+8
} type_PNDCR_FlightTask_Step; //16

/** 
  * @brief  структура полетного задания
  */
typedef struct
{
	type_PNDCR_FlightTask_Step step[128];
} type_PNDCR_FlightTask; //2048

/** 
  * @brief  структура управления полетным заданием
  */
typedef struct
{
	type_PNDCR_FlightTask default_flt;
	type_PNDCR_FlightTask can1;
	type_PNDCR_FlightTask can2;
	type_PNDCR_FlightTask work;
	uint8_t step_num;
	uint32_t pause_ms;
	uint16_t step_repeat_cnt;
} type_PNDCR_FlightTask_Ctrl;

/** 
  * @brief  структура формирования параметров для хранения в ПЗУ по Декор (18 байт) для последующей упаковки всех состаяний в один кадр из 116 байт
  */
typedef struct
{
	uint16_t mode; 				//+0
	uint16_t frame_cnt;		//+2
	uint16_t status_cnt;	//+4
	uint8_t inhibit;			//+6
	uint8_t gap;					//+7
	uint8_t rsrv[10]; 		//+8
} type_PNDCR_сfg; 			//18

#pragma pack(8)

/** 
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_PWR_CHANNEL *mcu_pwr_ch, *msr_pwr_ch;
	uint16_t pwr_check_timeout_ms;
	type_PNDCR_interface uart;
	type_PNDCR_report report;
	type_PNDCR_FlightTask_Ctrl fl_task;
	type_PNDCR_сfg loaded_cfg;
	type_PNDCR_сfg cfg;
	uint8_t self_num;
	uint16_t mode;
	uint16_t frame_cnt;
	uint16_t status_cnt;
	uint16_t status;
	uint16_t error_flags;
	uint8_t error_cnt;
	uint8_t inhibit;
	uint8_t last_received_frame[124], last_received_frame_leng;
	uint8_t last_received_status[116], last_received_status_leng;
	uint16_t rx_frames_cnt, rx_status_cnt;
} type_PN_DCR_model;

void pn_dcr_init(type_PN_DCR_model* pn_dcr_ptr, uint8_t num, UART_HandleTypeDef *uart_ptr, type_PWR_CHANNEL* mcu_pwr_ch_ptr, type_PWR_CHANNEL* msr_pwr_ch_ptr);
void pn_dcr_reset_state(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_process(type_PN_DCR_model* pn_dcr_ptr, uint16_t period_ms);
void pn_dcr_set_status(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_set_inh(type_PN_DCR_model* pn_dcr_ptr, uint8_t inh);
uint8_t pn_dcr_get_short_status(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_report_create(type_PN_DCR_model* pn11_ptr);
///*** функции поддержки работы с питанием ***///
void pn_dcr_pwr_process(type_PN_DCR_model* pn_dcr_ptr, uint16_t period_ms);
uint8_t pn_dcr_pwr_check(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_pwr_on(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
void pn_dcr_pwr_off(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
uint8_t pn_dcr_get_outputs_state(type_PN_DCR_model* pn11_ptr);
///*** Flight task ***///
void pn_dcr_flight_task_process(type_PN_DCR_model* pn_dcr_ptr, uint16_t period_ms);
void pn_dcr_set_mode(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
uint8_t pn_dcr_run_step_function(type_PN_DCR_model* pn_dcr_ptr);
uint8_t _pn_dcr_form_frame(uint8_t frame_type, type_PN_DCR_frame *frame, uint8_t cmd_type, uint8_t cmd_header, uint8_t *data);
uint8_t _pn_dcr_form_snc_time_frame(uint8_t frame_type, type_PN_DCR_frame *frame, uint32_t unix_time);
void pn_dcr_load_can_flight_task(type_PN_DCR_model* pn_dcr_ptr, uint8_t *flight_task, uint8_t fl_task_num);
void pn_dcr_fill_default_flight_task(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_fill_flight_task_step(type_PNDCR_FlightTask_Step* step, uint8_t type, uint8_t cmd, uint32_t pause_ms, uint16_t repeat_cnt, uint8_t *data);
///*** Cfg ***///
uint8_t pn_dcr_get_cfg(type_PN_DCR_model* pn_dcr_ptr, uint8_t *cfg);
uint8_t pn_dcr_set_cfg(type_PN_DCR_model* pn_dcr_ptr, uint8_t *cfg);
// работа с интерфейсом, исключительно передача и прием данных, без разбора и принятия решений по Декор
void pn_dcr_uart_init(type_PNDCR_interface *int_ptr, UART_HandleTypeDef *uart_ptr);
void pn_dcr_uart_send(type_PNDCR_interface *int_ptr, uint8_t* data, uint8_t len);
void pn_dcr_uart_rx_prcs_cb(type_PNDCR_interface *int_ptr);
void pn_dcr_uart_tx_prcs_cb(type_PNDCR_interface *int_ptr);
void pn_dcr_uart_err_prcs_cb(type_PNDCR_interface *int_ptr);
void _pn_dcr_uart_error_collector(type_PNDCR_interface *int_ptr, uint16_t error);
///*** Верхний уровень протокола общения с ДеКоР ***///
void pn_dcr_send_cmd(type_PN_DCR_model* pn_dcr_ptr, uint8_t cmd_type, uint8_t *data);
uint8_t pn_dcr_get_data(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
void pn_dcr_process_rx_frames(type_PN_DCR_model* pn_dcr_ptr, uint16_t pause_ms);
uint8_t pn_dcr_get_last_frame(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
uint8_t pn_dcr_get_last_status(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
///*** General perpose function ***///
void  _pn_dcr_error_collector(type_PN_DCR_model* pn_dcr_ptr, uint16_t error, int16_t data);
void _pn_dcr_fill_data_array(uint8_t*data, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7);
#endif
