#ifndef _PN_DCR_H
#define _PN_DCR_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "usart.h"
#include "ext_mem.h"

#include "rtc.h"

#define PN_DCR_PWR_MCU 0x01
#define PN_DCR_PWR_MSR 0x02
#define PN_DCR_PWR_ALL 0x03

#define PN_DCR_ERR_NO_ERROR 					(0x00)
#define PN_DCR_ERR_WRONG_FRAME_LENG 	(0x01 << 0)
#define PN_DCR_ERR_OTHER							(0x01 << 8)

#define PN_DCR_UART_ERR_NO_ERROR 			(0x00)
#define PN_DCR_UART_ERR_BAD_FRAME			(0x01 << 0)
#define PN_DCR_UART_ERR_HAL_NOISE			(0x01 << 1)
#define PN_DCR_UART_ERR_HAL_OTHER			(0x01 << 2)
#define PN_DCR_UART_ERR_OTHER					(0x01 << 8)

#define PN_DCR_UART_FRAME_SHORT				(0x01)
#define PN_DCR_UART_FRAME_LONG				(0x02)

#define PN_DCR_CMD_GET_TM_STATUS			(0x01)
#define PN_DCR_CMD_GET_DATA_MONITOR		(0x02)
#define PN_DCR_CMD_GET_DATA_MASSIVE		(0x03)


#define PN_DCR_CURR_MAX (2<<8)
#define PN_DCR_CURR_MIN (1<<0)
#define PN_DCR_VOLT_MAX (6<<8)
#define PN_DCR_VOLT_MIN (4<<8)
#define PN_DCR_PWR_MAX 	(10<<8)
#define PN_DCR_PWR_MIN 	(1<<0)

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

#pragma pack(8)

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
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_PWR_CHANNEL *mcu_pwr_ch, *msr_pwr_ch;
	type_PNDCR_interface uart;
	type_PNDCR_report report;
	uint16_t status, error_flags;
	uint8_t error_cnt;
	uint8_t last_received_frame[124], last_received_frame_leng;
	uint8_t last_received_status[116], last_received_status_leng;
	uint16_t rx_frames_cnt, rx_status_cnt;
} type_PN_DCR_model;

void pn_dcr_init(type_PN_DCR_model* pn_dcr_ptr, UART_HandleTypeDef *uart_ptr, type_PWR_CHANNEL* mcu_pwr_ch_ptr, type_PWR_CHANNEL* msr_pwr_ch_ptr);
void pn_dcr_reset_state(type_PN_DCR_model* pn_dcr_ptr);
void pn_dcr_report_create(type_PN_DCR_model* pn11_ptr);
uint8_t pn_dcr_get_outputs_state(type_PN_DCR_model* pn11_ptr);
void pn_dcr_pwr_on(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
void pn_dcr_pwr_off(type_PN_DCR_model* pn_dcr_ptr, uint8_t mode);
void pn_dcr_send_cmd(type_PN_DCR_model* pn_dcr_ptr, uint8_t cmd_type, uint8_t *data);
uint8_t pn_dcr_get_data(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
void pn_dcr_process_rx_frames_10ms(type_PN_DCR_model* pn_dcr_ptr);
uint8_t pn_dcr_get_last_frame(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
uint8_t pn_dcr_get_last_status(type_PN_DCR_model* pn_dcr_ptr, uint8_t *data);
void pn_dcr_process_flight_task_100ms(type_PN_DCR_model* pn_dcr_ptr);
uint8_t _pn_dcr_form_frame(uint8_t frame_type, type_PN_DCR_frame *frame, uint8_t cmd_type, uint8_t cmd_header, uint8_t *data);
void  _pn_dcr_error_collector(type_PN_DCR_model* pn_dcr_ptr, uint16_t error);

// работа с интерфейсом, исключительно передача и прием данных, без разоора и принятия решений по Декор
void pn_dcr_uart_init(type_PNDCR_interface *int_ptr, UART_HandleTypeDef *uart_ptr);
void pn_dcr_uart_send(type_PNDCR_interface *int_ptr, uint8_t* data, uint8_t len);
void pn_dcr_uart_rx_prcs_cb(type_PNDCR_interface *int_ptr);
void pn_dcr_uart_tx_prcs_cb(type_PNDCR_interface *int_ptr);
void pn_dcr_uart_err_prcs_cb(type_PNDCR_interface *int_ptr);
void _pn_dcr_uart_error_collector(type_PNDCR_interface *int_ptr, uint16_t error);

#endif
