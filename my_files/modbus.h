
#ifndef _MODBUS_H
#define _MODBUS_H

#include "usart.h"
#include "ext_mem.h"
#include "clock.h"

// дефайны для переменных
#define MB_UART_TIMEOUT_MS								20

#define MB_UART_RX_STATUS_OK							1
#define MB_UART_RX_STATUS_SHORT						0
#define MB_UART_RX_STATUS_LENGTH					-1
#define MB_UART_CRC_ERROR									-2
#define MB_UART_UNEXPECTED_FRAME					-3

#define MB_CRC16_INIT_VAL 0xFFFF

#define MB_FRAME_TYPE_RX 0x01
#define MB_FRAME_TYPE_TX 0x02
#define MB_FRAME_TYPE_ERROR 0x03

#define MB_ERROR_TYPE_EXCEPTION					0x00
#define MB_ERROR_TYPE_WRONG_FC					0x01
#define MB_ERROR_TYPE_WRONG_REG_ADDR 		0x02
#define MB_ERROR_TYPE_WRONG_REG_DATA 		0x03
#define MB_ERROR_TYPE_CRITICAL					0x04
#define MB_ERROR_TYPE_WRONG_CRC					0x08

#define MB_F_CODE_3											0x03
#define MB_F_CODE_6											0x06
#define MB_F_CODE_16										0x16
#define MB_F_CODE_ERROR									0x80

#define MB_TRANSACTION_WRITE				0x02
#define MB_TRANSACTION_READ					0x01
#define MB_TRANSACTION_IDLE 				0x00

#define MB_QEUEU_START				0x01
#define MB_QEUEU_WORK					0x02
#define MB_QUUE_IDLE 					0x00

#pragma pack(2)

typedef struct{
	uint8_t dev_id;
	uint8_t f_code;
	uint16_t reg_addr;
	uint16_t reg_cnt;
	uint8_t byte_cnt;
	uint16_t data[256];
	uint16_t crc_16;
	uint8_t error_code;
	uint8_t type;
} type_MB_Frame;

typedef struct{
	type_MB_Frame rx_frame[8];
	type_MB_Frame tx_frame[8];
} type_MB_Queue;
/** 
  * @brief  структура хранения параметров информационного интерфейса
  */
typedef struct
{
	UART_HandleTypeDef* huart;
	type_MB_Queue work_queue, prepare_queue;
	type_MB_Frame rx_frame, tx_frame;
	uint8_t tx_data[256], tx_len, tx_cnt;
	uint8_t rx_data[256], rx_len, rx_cnt, rx_ptr, rx_ptr_offset, row_rx_data[256], row_rx_len;
	uint8_t transaction_state, queue_state, queue_cnt;
	uint16_t rx_timeout;
	uint8_t rx_timeout_flag, rx_data_ready;
	// 
	// uint8_t sw_tmi_comm[PN_20_SW_TEST_U16_LENG];
	// uint8_t tas_start_comm[PN_20_TAS_START_U16_LENG];
	// uint8_t tas_result_comm[PN_20_TAS_RESULT_U16_LENG];
	// uint8_t sd_start_comm[PN_20_SD_START_U16_LENG];
	// uint8_t sd_result_comm[PN_20_SD_RESULT_U16_LENG];
	// uint8_t comm_queue_array[64], comm_queue_cnt, comm_queue_flg, comm_queue_leng;
	// uint16_t queue_interval_ms, queue_step_time_ms;
	// uint16_t *array_to_save;
	// uint16_t array_to_save_trash;
	// uint8_t array_to_save_index;
	// //
	uint8_t error_flags;
	uint8_t error_cnt;
} type_MODBUS;

#pragma pack(8)

void mb_init(type_MODBUS* mb_ptr, UART_HandleTypeDef *uart_ptr);
void mb_process(type_MODBUS* mb_ptr, uint16_t period_ms);
void mb_rx_ptr_reset(type_MODBUS* mb_ptr);
int8_t mb_run_transaction(type_MODBUS* mb_ptr, uint8_t dev_id, uint8_t f_code, uint16_t reg_addr, uint16_t reg_cnt, uint16_t* data);
void mb_process_transaction(type_MODBUS* mb_ptr, uint32_t period_ms);
int8_t mb_send_frame(type_MODBUS* mb_ptr, uint8_t dev_id, uint8_t f_code, uint16_t reg_addr, uint16_t reg_cnt, uint16_t* data);
int8_t mb_get_frame(type_MODBUS* mb_ptr, uint8_t* data, uint8_t leng, uint8_t frame_type);

int8_t mb_int_check_frame(type_MODBUS* mb_ptr, uint8_t* data, uint8_t len);
void mb_int_rx_huart_cb(type_MODBUS* mb_ptr);
void mb_rx_timeout_cb(type_MODBUS* mb_ptr, uint16_t period_ms);
int8_t mb_int_check_frame(type_MODBUS* mb_ptr, uint8_t* data, uint8_t len);

void mb_queue_fill(type_MODBUS* mb_ptr, uint8_t step_num, uint8_t dev_id, uint8_t f_code, uint16_t reg_addr, uint16_t reg_cnt, uint16_t* data);
int8_t mb_queue_run(type_MODBUS* mb_ptr);
int8_t mb_queue_process(type_MODBUS* mb_ptr, uint32_t period_ms);


void mb_frame_init(type_MB_Frame* frame_ptr);
uint16_t mb_frame_calc_crc16(type_MB_Frame* frame_ptr);
void mb_frame_form_packet(type_MB_Frame* frame_ptr, uint8_t* data, uint8_t* leng);

uint16_t __mb_crc16(uint8_t *data, uint16_t len, uint16_t crc16_init);

#endif
