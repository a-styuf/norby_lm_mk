/**
  ******************************************************************************
  * @file           : pn_11_interface_app_lvl.c
  * @version        : v1.0
  * @brief          : реализация упрвня приложения для ПН1.1
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn_11_interface_app_lvl.h"

/**
  * @brief  инийиализация протокола уровня приложения
  * @param  app_lvl_ptr: указатель на структуру управления уровнем приложения
  * @param  huart: указатель на структуру UART
  */
void app_lvl_init(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, UART_HandleTypeDef* huart)
{
	tr_lvl_init(&app_lvl_ptr->tr_lvl, huart);
}

/**
  * @brief  запись данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  huart: указатель на структуру UART
  */
void app_lvl_write(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len)
{
	
}

/**
  * @brief  запрос на чтение данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  huart: указатель на структуру UART
  */
void app_lvl_read_req(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr, uint32_t addr, uint32_t* data, uint8_t len)
{
	
}

/**
  * @brief  команда на чтение данных
  * @param  app_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @retval  status: 1 - чтение удалось, 0 - чтение в ожидании ответа, <0 - ошибка чтения
  */
int8_t app_lvl_read_check(type_PN11_INTERFACE_APP_LVL* app_lvl_ptr)
{
	return 0;
}
