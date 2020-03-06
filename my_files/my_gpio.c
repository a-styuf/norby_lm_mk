/**
  ******************************************************************************
  * @file           : my_gpio.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX для организации удобной работы с gpio
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "my_gpio.h"

type_GPIO_setting gpio_parameters_set(GPIO_TypeDef* bank, uint16_t position)
{
	type_GPIO_setting gpio;
	gpio.bank = bank;
	gpio.position = position;
	gpio.state = 0x00;
	return gpio;
}

void gpio_set(type_GPIO_setting* gpio_ptr, uint8_t value)
{
	uint16_t var;
	var = (uint16_t)(1<<(gpio_ptr->position & 0x0F));
	HAL_GPIO_WritePin(gpio_ptr->bank, var, (GPIO_PinState)(value&0x01));
}

uint8_t gpio_get(type_GPIO_setting* gpio_ptr)
{
	uint16_t var;
	var = (uint16_t)(1<<(gpio_ptr->position & 0x0F));
	gpio_ptr->state = (uint8_t)HAL_GPIO_ReadPin(gpio_ptr->bank, var);
	return gpio_ptr->state & 0x01;
}
