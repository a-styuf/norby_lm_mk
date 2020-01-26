#ifndef __LM_H
#define __LM_H

#include "my_gpio.h"
#include "i2c.h"
#include "crc16.h"
#include "math.h"
#include "ina226.h"
#include <stdlib.h>

#define max(A, B) ((A) > (B) ? (A) : (B))
#define min(A, B) ((A) < (B) ? (A) : (B))

typedef unsigned short uint16_t;

// настройки прибора
#define DEV_ID (0x01)

// свойства микроконтроллера

// раскрашивание переменных

// описание рабочих структур
typedef struct
{
	type_INA226_DEVICE ina226;
	type_GPIO_setting ena[3];
} type_PWR_CHANNEL;

typedef struct
{ 
	type_PWR_CHANNEL ch[7]; // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
	type_GPIO_setting gd, alert;
	I2C_HandleTypeDef* i2c_ptr;
} type_PWR_CONTROL;

typedef struct
{
	uint32_t global_time_s;
	type_PWR_CONTROL pwr;
} type_LM_DEVICE;

// прототипы функций

//*** ITB ***//
void lm_init(type_LM_DEVICE* lm_ptr);
void pwr_init(type_PWR_CONTROL* pwr_ptr, I2C_HandleTypeDef* hi2c_ptr);
void pwr_ch_on_off(type_PWR_CONTROL* pwr_ptr, uint8_t channel_num, uint8_t mode);
void pwr_on_off(type_PWR_CONTROL* pwr_ptr, uint8_t pwr_switches);

uint16_t com_ans_form(uint8_t req_id, uint8_t self_id, uint8_t* seq_num, uint8_t type, uint8_t leng, uint8_t* com_data, uint8_t* ans_com);
#endif
