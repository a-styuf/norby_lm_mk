#ifndef _PWR_CH_H
#define _PWR_CH_H

#include "ina226.h"
#include "my_gpio.h"
#include "debug.h"

//режим канала
#define PWR_CH_OFF					0x00
#define PWR_CH_ON		 				0x01

//типы ошибки питания
#define PWR_CH_ERR_NO_ERR		(0x00)
#define PWR_CH_ERR_VOLTAGE	(0x01 << 0)
#define PWR_CH_ERR_PWR			(0x01 << 1)
#define PWR_CH_ERR_SWITCH		(0x01 << 2)
#define PWR_CH_ERR_OTHER		(0x01 << 3)


typedef struct
{
	type_INA226_DEVICE ina226;
	type_GPIO_setting ena[3];
	float voltage_max, voltage_min;
	float power_max, power_min;
	uint8_t mode;
	uint8_t error;
	uint8_t error_check_inh;
} type_PWR_CHANNEL;

int8_t pwr_ch_init(type_PWR_CHANNEL* pwr_ch_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t i2c_addr, uint16_t power_lim_Wt, GPIO_TypeDef* ena1_bank, uint16_t ena1_pos, GPIO_TypeDef* ena2_bank, uint16_t ena2_pos, GPIO_TypeDef* ena3_bank, uint16_t ena3_pos);
void pwr_ch_set_bound(type_PWR_CHANNEL* pwr_ch_ptr, float u_max, float u_min, float pow_max, float pow_min);
uint8_t pwr_ch_get_error(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t *error);
void pwr_ch_on_off(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t mode); //работает для всех каналов, кроме 0-го (МС) - он всегда включен
void pwr_ch_on_off_separatly(type_PWR_CHANNEL* pwr_ch_ptr, uint8_t mode);

#endif
