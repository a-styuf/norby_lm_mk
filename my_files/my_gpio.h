#ifndef _MY_GPIO_H
#define _MY_GPIO_H

#include "gpio.h"

/** 
  * @brief  структура хранения всех параметров gpio достаточных для управления
  */
typedef struct
{
	GPIO_TypeDef* bank;
	uint16_t position;
	uint8_t state;
} type_GPIO_setting;

type_GPIO_setting gpio_parameters_set(GPIO_TypeDef* bank, uint16_t position);
void gpio_set(type_GPIO_setting* gpio_ptr, uint8_t value);
uint8_t gpio_get(type_GPIO_setting* gpio_ptr);
#endif
