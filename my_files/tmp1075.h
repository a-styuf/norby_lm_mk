#ifndef _TMP1075_H
#define _TMP1075_H

#include "i2c.h"
#include "string.h"

// настройки TMP1075
// адреса регистров
#define TEMP_REGISTER_ADDR 0x00
#define CFGR_ADDR 0x01 //config register
#define LLIM_REGISTER_ADDR 0x02
#define HLIM_REGISTER_ADDR 0x03
#define DEV_ID_REGISTER_ADDR 0x0F

//Configuration_register
#define AVEREGES_DEF AVERAGES_NUM_1024
#define CONV_TIME_DEF CONV_TIME_1100us
#define MODE_DEF SHUNT_VOLTAGE_CONT
//Default condiguration
#define CONFIG_DEFAULT ((0x00<<15)|(0x01<<13)|(0x02<<11)|(0x00<<10)|(0x00<<9)|(0x00<<8)|(0xFF<<0)) // continuos mode, pweiod - 110ms, alert-active low
#define LLIM_DEFAULT (85*256)  //85°C
#define HLIM_DEFAULT (80*256)  //80°C
#define TMP_DEV_ID 0x0075

//типы ошибки температуры
#define TMP_CH_ERR_NO_ERR		(0x00)
#define TMP_CH_ERR_TOO_HIGH	(0x01 << 0)
#define TMP_CH_ERR_TOO_LOW	(0x01 << 1)
#define TMP_CH_ERR_SENSOR		(0x01 << 2)
#define TMP_CH_ERR_OTHER		(0x01 << 3)

typedef struct
{
	I2C_HandleTypeDef* i2c_ptr;
	uint8_t addr;
	uint8_t tx_data[4];
	uint16_t rx_data;
	uint8_t rx_reg_addr;
	uint8_t validate_data[16];
	int16_t temp;
	int16_t temp_high, temp_low;
	uint8_t queue_state;
	uint8_t error;
} type_TMP1075_DEVICE;

uint8_t tmp1075_init(type_TMP1075_DEVICE* tmp1075_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t addr);
uint8_t tmp1075_alert_lvl_set(type_TMP1075_DEVICE* tmp1075_ptr, int16_t temp_high,  int16_t temp_low);
uint8_t tmp1075_reg_addr_set(type_TMP1075_DEVICE* tmp1075_ptr, uint8_t reg_addr);
void tmp1075_set_bound(type_TMP1075_DEVICE* tmp1075_ptr, int16_t temp_high, int16_t temp_low);
uint16_t tmp1075_read_request(type_TMP1075_DEVICE* tmp1075_ptr);
uint16_t tmp1075_read_data_process(type_TMP1075_DEVICE* tmp1075_ptr);
void tmp1075_start_read_queue(type_TMP1075_DEVICE* tmp1075_ptr);
void tmp1075_body_read_queue(type_TMP1075_DEVICE* tmp1075_ptr);
void tmp1075_error_process(type_TMP1075_DEVICE* tmp1075_ptr);
uint8_t tmp1075_get_error(type_TMP1075_DEVICE* tmp1075_ptr, uint8_t *error);
#endif
