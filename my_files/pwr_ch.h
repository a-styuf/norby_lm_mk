#ifndef _PWR_CH_H
#define _PWR_CH_H

#include "ina226.h"
#include "my_gpio.h"

typedef struct
{
	type_INA226_DEVICE ina226;
	type_GPIO_setting ena[3];
} type_PWR_CHANNEL;

void pwr_ch_init(type_PWR_CHANNEL* pwr_ch_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t i2c_addr, uint16_t power_lim_Wt, GPIO_TypeDef* ena1_bank, uint16_t ena1_pos, GPIO_TypeDef* ena2_bank, uint16_t ena2_pos, GPIO_TypeDef* ena3_bank, uint16_t ena3_pos);
void pwr_ch_on_off(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t mode); //работает для всех каналов, кроме 0-го (МС) - он всегда включен

#endif
