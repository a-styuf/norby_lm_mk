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
#define APP_LVL_NANS_ERROR      (0x1 << 2)

#define APP_LVL_ADDR_MODE     0x0

#pragma pack(1)
/** 
  * @brief  структура управлени протоколом уровня приложения
  */
typedef struct
{
	uint8_t ctrl_byte;
  uint32_t addr;
  uint32_t data[61];
} type_APP_LVL_PCT;

#pragma pack(8)
/** 
  * @brief  структура управлени протоколом уровня приложения
  */
typedef struct
{
	uint32_t mode;
  uint32_t packet[3][240];
  uint32_t pr_tmi[3];
} type_PN11_MEM_STR;

/** 
  * @brief  структура управлени протоколом уровня приложения
  */
typedef struct
{
	type_PN11_INTERFACE_TR_LVL tr_lvl;
  uint32_t error; //битовая маска: 0-ошибка длины данных, 1-
  type_APP_LVL_PCT wr_frame;
  type_APP_LVL_PCT rd_frame;
} type_PN11_INTERFACE_APP_LVL;

void app_lvl_init(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, UART_HandleTypeDef* huart);
int8_t app_lvl_write(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len);
void _app_lvl_form_data(type_APP_LVL_PCT *frame_ptr, uint8_t data_len, uint8_t *u8_data, uint8_t *u8_len_ptr);

void app_lvl_process_10ms(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr);
int8_t app_lvl_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint8_t len);
int8_t app_lvl_read_check(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr);
#endif
