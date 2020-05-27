#ifndef _INA226_H
#define _INA226_H

#include "i2c.h"
#include "string.h"

// Параметры усреднения
#define AVERAGES_NUM_1 0x00
#define AVERAGES_NUM_4 0x01
#define AVERAGES_NUM_16 0x02
#define AVERAGES_NUM_64 0x03
#define AVERAGES_NUM_128 0x04
#define AVERAGES_NUM_256 0x05
#define AVERAGES_NUM_512 0x06
#define AVERAGES_NUM_1024 0x07
//Время преобразования для тока или напряжения
#define CONV_TIME_140us 0x00
#define CONV_TIME_204us 0x01
#define CONV_TIME_332us 0x02
#define CONV_TIME_588us 0x03
#define CONV_TIME_1100us 0x04
#define CONV_TIME_2116us 0x05
#define CONV_TIME_4156us 0x06
#define CONV_TIME_8244us 0x07
//режимы оперирования
#define SHUTDOWN 0x00
#define SHUNT_TRIG 0x01
#define VOLTAGE_TRIG 0x02
#define SHUNT_VOLTAGE_TRIG 0x03
#define SHUTDOWN_ALT 0x04
#define SHUNT_CONT 0x05
#define VOLTAGE_CONT 0x06
#define SHUNT_VOLTAGE_CONT 0x07
// проверочные регистры

// настройки INA226
// адреса регистров
#define CONFIG_REGISTER_ADDR 0x00
#define VBUS_REGISTER_ADDR 0x02
#define CURRENT_REGISTER_ADDR 0x04
#define POWER_REGISTER_ADDR 0x03
#define CAL_REGISTER_ADDR 0x05
#define ALERT_REGISTER_ADDR 0x07
#define MODE_REGISTER_ADDR 0x06
#define MANUFACTURED_ID_REG_ADDR 0xFE
#define DIE_ID_REG_ADDR 0xFF

//Configuration_register
#define AVEREGES_DEF AVERAGES_NUM_128
#define CONV_TIME_DEF CONV_TIME_588us
#define MODE_DEF SHUNT_VOLTAGE_CONT
#define ALERT_MODE_DEF (1<<11)
//Calibration register: This value is needed to recalculate for each shunt
#define CURRENT_LSB (0.001)  //A
#define CURRENT_R_SHUNT (0.015)  //Ом
#define CAL_VAL (0.00512/(CURRENT_LSB*CURRENT_R_SHUNT))
#define POWER_KOEFF_FROM_WT_TO_ADC (0.04/CURRENT_LSB)

typedef struct
{
	I2C_HandleTypeDef* i2c_ptr;
	uint8_t addr;
	uint8_t tx_data[4];
	uint16_t rx_data;
	uint8_t rx_reg_addr;
	uint8_t validate_data[16];
	uint16_t voltage, current, power;
	uint8_t queue_state; 
	uint8_t error_cnt;
} type_INA226_DEVICE;

uint8_t ina226_init(type_INA226_DEVICE* ina226_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t addr, uint16_t power_lim_Wt);
uint16_t ina226_read_request(type_INA226_DEVICE* ina226_ptr);
uint16_t ina226_read_IT_callback(type_INA226_DEVICE* ina226_ptr);
void ina226_start_read_queue(type_INA226_DEVICE* ina226_ptr);
void ina226_body_read_queue(type_INA226_DEVICE* ina226_ptr);
void ina226_error_process(type_INA226_DEVICE* ina226_ptr);

#endif
