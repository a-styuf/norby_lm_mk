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
#define CONFIG_DEFAULT (0x01<<15)|(0x01<<13)|(0x02<<11)|(0x00<<10)|(0x00<<9)|(0x00<<8)|(0x00<<0) // continuos mode, pweiod - 110ms, alert-active low
#define LLIM_DEFAULT (55<<8)  //55°C
#define HLIM_DEFAULT (60<<8)  //60°C
#define TMP_DEV_ID 0x7500

typedef struct
{
	I2C_HandleTypeDef* i2c_ptr;
	uint8_t addr;
	uint8_t tx_data[4];
	uint16_t rx_data;
	uint8_t rx_reg_addr;
	uint8_t validate_data[8];
	uint16_t temp;
	uint16_t temp_high, temp_low;
	uint8_t queue_state;
	uint8_t error_cnt;
} type_TMP1075_DEVICE;

uint8_t tmp1075_init(type_TMP1075_DEVICE* tmp1075_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t addr);
uint8_t tmp1075_alert_lvl_set(type_TMP1075_DEVICE* tmp1075_ptr, uint16_t temp_high,  uint16_t temp_low);
uint8_t tmp1075_reg_addr_set(type_TMP1075_DEVICE* tmp1075_ptr, uint8_t reg_addr);
uint16_t tmp1075_read_request(type_TMP1075_DEVICE* tmp1075_ptr);
uint16_t tmp1075_read_data_process(type_TMP1075_DEVICE* tmp1075_ptr);
void tmp1075_start_read_queue(type_TMP1075_DEVICE* tmp1075_ptr);
void tmp1075_body_read_queue(type_TMP1075_DEVICE* tmp1075_ptr);
void tmp1075_error_process(type_TMP1075_DEVICE* tmp1075_ptr);
#endif
