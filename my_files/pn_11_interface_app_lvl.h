#ifndef _PN_11_INTERFACE_APP_LVL_H
#define _PN_11_INTERFACE_APP_LVL_H

#include "usart.h"
#include "crc16.h"
#include "pn_11_interface_tr_lvl.h"

/** 
  * @brief  структура управлени протоколом уровня приложения
  */
typedef struct
{
	type_PN11_INTERFACE_TR_LVL tr_lvl;
} type_PN11_INTERFACE_APP_LVL;

void app_lvl_init(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, UART_HandleTypeDef* huart);
void app_lvl_write(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len);
void app_lvl_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len);
int8_t app_lvl_read_check(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr);
#endif
