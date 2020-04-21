#ifndef _PN_DCR_H
#define _PN_DCR_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "usart.h"
#include "ext_mem.h"

#include "rtc.h"

#define DCR_MODE_OFF 								0
#define DCR_MODE_DEFAULT 						1
#define DCR_MODE_FLIGHT_TASK 				2
#define DCR_MODE_PAUSE			 				3

#define DCR_PRELODED_FL_TASK_DEFAULT			0x00
#define DCR_PRELODED_FL_TASK_OTHER				0x01
#define DCR_PRELODED_FL_TASK_FROM_CAN			0xFF

// типы команды для полетного задания
#define DCR_PN_FLT_TYPE_PWR 							0x01
#define DCR_PN_FLT_TYPE_UART							0x02


#define PN_DCR_PWR_MCU			 0x01
#define PN_DCR_PWR_MSR			 0x02
#define PN_DCR_PWR_ALL			 0x03

#define PN_DCR_ERR_NO_ERROR 					(0x00)
#define PN_DCR_ERR_WRONG_FRAME_LENG 	(0x01 << 0)
#define PN_DCR_ERR_FL_TASK_ERROR			(0x01 << 1)
#define PN_DCR_ERR_WRONG_MODE					(0x01 << 2)
#define PN_DCR_ERR_OTHER							(0x01 << 3)
#define PN_DCR_ERR_PWR								(0x01 << 4)
#define PN_DCR_ERR_UART								(0x01 << 12)

#define PN_DCR_UART_ERR_NO_ERROR 			(0x00)
#define PN_DCR_UART_ERR_BAD_FRAME			(0x01 << 0)
#define PN_DCR_UART_ERR_HAL_NOISE			(0x01 << 1)
#define PN_DCR_UART_ERR_HAL_OTHER			(0x01 << 2)
#define PN_DCR_UART_ERR_OTHER					(0x01 << 3)

#define PN_DCR_UART_FRAME_SHORT				(0x01)
#define PN_DCR_UART_FRAME_LONG				(0x02)

#define PN_DCR_CMD_GET_TM_STATUS			(0x01)
#define PN_DCR_CMD_GET_DATA_MONITOR		(0x02)
#define PN_DCR_CMD_GET_DATA_MASSIVE		(0x03)

// пороговые значения для определения ошибки питания: напряжение в В, ток в А, мощность в Вт

// пороги напряжения общие для двух каналов
#define PN_DCR_VOLT_MAX 		9.0
#define PN_DCR_VOLT_MIN 		4.5
// границы мощности для линии питания МК
#define PN_DCR_MC_PWR_MAX 	0.4
#define PN_DCR_MC_PWR_MIN 	0.1
// границы мощности для лини питания датчика
#define PN_DCR_MSR_PWR_MAX 	1.6
#define PN_DCR_MSR_PWR_MIN 	0.4
// задержка для определения ошибки питания - устанавливается каждый ра, когда происходит изменение состояния питания
#define PN_DCR_PWR_TIMEOUT_MS 	1000

//Ошибки работы
#define PN_DCR_STATE_NO_ERROR				(0)


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
	uint16_t rsrv; //+6
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
	type_PNDCR_FlightTask can;
	type_PNDCR_FlightTask work;
	uint8_t step_num;
	uint32_t pause_ms;
} type_PNDCR_FlightTask_Ctrl;

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
	uint16_t status;
	uint16_t error_flags;
	uint8_t error_cnt;
	uint8_t last_received_frame[124], last_received_frame_leng;
	uint8_t last_received_status[116], last_received_status_leng;
	uint16_t rx_frames_cnt, rx_status_cnt;
} type_PN_DCR_model;

void pn_dcr_init(type_PN_DCR_model* pn_dcr_ptr, UART_HandleTypeDef *uart_ptr, type_PWR_CHANNEL* mcu_pwr_ch_ptr, type_PWR_CHANNEL* msr_pwr_ch_ptr);
void pn_dcr_fill_default_flight_task(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_fill_flight_task_step(type_PNDCR_FlightTask_Step* step, uint8_t type, uint8_t cmd, uint32_t pause_ms, uint8_t *data);
void _pn_dcr_fill_data_array(uint8_t*data, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7);
void pn_dcr_reset_state(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_report_create(type_PN_DCR_model* pn11_ptr);
uint8_t pn_dcr_get_outputs_state(type_PN_DCR_model* pn11_ptr);
//
void pn_dcr_pwr_on(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
void pn_dcr_pwr_off(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
uint8_t pn_dcr_pwr_check(type_PN_DCR_model* pn_dcr_ptr, uint16_t timeout);
//
void pn_dcr_send_cmd(type_PN_DCR_model* pn_dcr_ptr, uint8_t cmd_type, uint8_t *data);
uint8_t pn_dcr_get_data(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
void pn_dcr_process_rx_frames_10ms(type_PN_DCR_model* pn_dcr_ptr);
uint8_t pn_dcr_get_last_frame(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
uint8_t pn_dcr_get_last_status(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
void pn_dcr_set_mode(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
void pn_dcr_load_can_flight_task(type_PN_DCR_model* pn_dcr_ptr, uint8_t *flight_task);
void pn_dcr_process(type_PN_DCR_model* pn_dcr_ptr, uint32_t time_step_ms);
uint8_t pn_dcr_run_step_function(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_process_flight_task_100ms(type_PN_DCR_model* pn_dcr_ptr);
uint8_t _pn_dcr_form_frame(uint8_t frame_type, type_PN_DCR_frame *frame, uint8_t cmd_type, uint8_t cmd_header, uint8_t *data);
void  _pn_dcr_error_collector(type_PN_DCR_model* pn_dcr_ptr, uint16_t error, int16_t data);

// работа с интерфейсом, исключительно передача и прием данных, без разоора и принятия решений по Декор
void pn_dcr_uart_init(type_PNDCR_interface *int_ptr, UART_HandleTypeDef *uart_ptr);
void pn_dcr_uart_send(type_PNDCR_interface *int_ptr, uint8_t* data, uint8_t len);
void pn_dcr_uart_rx_prcs_cb(type_PNDCR_interface *int_ptr);
void pn_dcr_uart_tx_prcs_cb(type_PNDCR_interface *int_ptr);
void pn_dcr_uart_err_prcs_cb(type_PNDCR_interface *int_ptr);
void _pn_dcr_uart_error_collector(type_PNDCR_interface *int_ptr, uint16_t error);

#endif