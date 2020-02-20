#ifndef _PN_11_INTERFACE_TR_LVL_H
#define _PN_11_INTERFACE_TR_LVL_H

#include "usart.h"
#include "crc16.h"

#define FR_SPACE_REQ 0x00
#define FR_SPACE_ANS 0x02
#define FR_LAST_STATUS_REQ 0x01
#define FR_LAST_STATUS_ANS 0x06
#define FR_RST_REQ 0x03
#define FR_DATA 0x04

//типы статусов функций
#define NO_RECOGNISED_FRAME 127 // введен, так как 0 занят на FR_SPACE_REQ (ноль бы был уместней)
#define INCORRECT_DATA_CRC8 126
#define INCORRECT_DATA_NUM 125
#define CORRECT_DATA 1

#define DT_MAX_LEN (256-5)

#define ERR_TYPE_OK 0x00
#define ERR_TYPE_HEADER_CRC 0x02
#define ERR_TYPE_CRC 0x02
#define ERR_TYPE_SPACE 0x04
#define ERR_TYPE_NUM 0x06
#define ERR_TYPE_OTHERS 0xFF
#define ERR_TYPE_DATA_CRC 0x12

// типы данных
#define FRAME 0x01
#define DATA 0x02

// статусы передачи
#define TX_OK 0x00
#define CRC_HEADER_ERROR 0x01
#define CRC_DATA_ERROR 0x02
#define NUME_ERROR 0x04
#define SPACE_ERROR 0x08

/** 
  * @brief  структура хранения отчета по ПН1.1
  */
typedef struct
{
	UART_HandleTypeDef* huart;
	
	uint8_t rx_data[256];
	uint8_t rx_len;
	uint8_t rx_space;
	uint8_t rx_data_frame_num;
	uint8_t rx_frame_type;
	uint8_t rx_error_type;
	uint8_t rx_error_frame_num;
	uint8_t rx_error_code;
	uint8_t rx_error_flag;
	uint8_t rx_error_cnt;
	uint8_t rx_state;
	
	uint8_t tx_data[256];
	uint8_t tx_len;
	uint8_t tx_space;
	uint8_t tx_data_frame_num;
	uint8_t tx_frame_type;
	uint8_t tx_error_type;
	uint8_t tx_error_frame_num;
	uint8_t tx_error_code;
	uint8_t tx_error_flag;
	uint8_t tx_error_cnt;
	uint8_t tx_last_sended_type;
	uint8_t tx_state;
	
	uint8_t row_tx_data[256];
	uint8_t row_tx_len;
	uint8_t row_rx_data[256];
	uint8_t row_rx_len;
} type_PN11_INTERFACE_TR_LVL;

void tr_lvl_init(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, UART_HandleTypeDef* huart);
uint8_t tr_lvl_send_data(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t* data, uint8_t len);
void tr_lvl_process_10ms(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr);
void tx_create_frame(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t fr_type, uint8_t* data, uint8_t len);
void tx_create_data_frame(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t* data, uint8_t len);
uint8_t tx_get_error_type(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr);

int8_t rx_check_frame(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr);
void rx_error_set(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t type);
void rx_error_check(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t err_type);
uint8_t rx_data_check(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr);

void tx_uart_data(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr);
void rx_uart_data(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr);
#endif
