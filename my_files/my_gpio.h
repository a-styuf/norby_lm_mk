#ifndef _MY_GPIO_H
#define _MY_GPIO_H

#include "gpio.h"

// размер приемных и передающих пакетов
#define APP_RX_DATA_SIZE (64)
#define APP_TX_DATA_SIZE (64)

/** 
  * @brief  структура хранения всех параметров gpio достаточных для управления
  */
typedef struct
{
	GPIO_TypeDef* bank;
	uint16_t position;
} type_GPIO_setting;

type_GPIO_setting gpio_parameters_set(GPIO_TypeDef* bank, uint16_t position);
void gpio_set(type_GPIO_setting* gpio_ptr, uint8_t value);
#endif
