#ifndef _PN_11_INTERFACE_APP_LVL_H
#define _PN_11_INTERFACE_APP_LVL_H

#include "usart.h"
#include "crc16.h"
#include "pn_11_interface_tr_lvl.h"

// раскрашивание переменных
#define APP_LVL_MODE_READ     0x2
#define APP_LVL_MODE_WRITE    0x3

#define APP_LVL_NO_ERR          (0x0)
#define APP_LVL_ADDR_ERROR      (0x1 << 0)
#define APP_LVL_DATA_ERROR      (0x1 << 1)
#define APP_LVL_TIMEOUT_ERROR   (0x1 << 2)
#define APP_LVL_OTHER_ERROR     (0x1 << 3)

#define APP_LVL_MAX_U32_DATA         30  //принципиальное ограничение из-за работы через 128-байтные массивы

#define APP_LVL_DEFAULT_TIMEOUT_MS     100


#pragma pack(2)
/** 
  * @brief  структура управлени протоколом уровня приложения
  */
typedef struct
{
  uint32_t addr;
  uint32_t data[APP_LVL_MAX_U32_DATA];
	uint32_t ctrl_byte;
} type_APP_LVL_PCT; //128

#pragma pack(8)

/** 
  * @brief  структура управлени протоколом уровня приложения
  */
typedef struct
{
	type_PN11_INTERFACE_TR_LVL tr_lvl;
  uint8_t error_flags;
  uint8_t error_cnt;
  type_APP_LVL_PCT wr_frame;
  type_APP_LVL_PCT rd_frame;
  uint16_t rx_timeout;
  uint8_t rx_valid_flag; // данные приняты и готовы для чтения
  uint8_t rx_ready_flag; // данные приняты и еще не читались
  uint8_t rx_timeout_flag;
} type_PN11_INTERFACE_APP_LVL;

void app_lvl_init(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, UART_HandleTypeDef* huart);
void app_lvl_reset(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr);
void app_lvl_parameters_default(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr);
int8_t app_lvl_write(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len);
void _app_lvl_form_data(type_APP_LVL_PCT *frame_ptr, uint8_t data_len, uint8_t *u8_data, uint8_t *u8_len_ptr);

void app_lvl_process(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint16_t period_ms);
int8_t app_lvl_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint8_t len);
int8_t app_lvl_read_check(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr);
uint8_t app_lvl_get_last_rx_frame(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint8_t *last_data);
uint8_t app_lvl_get_rx_frame(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, type_APP_LVL_PCT *frame);


void  _app_lvl_error_collector(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint16_t error);
#endif
