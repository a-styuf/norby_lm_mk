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
#define MANUFACTURED_ID_REG_ADDR 0xFE
#define DIE_ID_REG_ADDR 0xFF

//Configuration_register
#define AVEREGES_DEF AVERAGES_NUM_4
#define CONV_TIME_DEF CONV_TIME_1100us
#define MODE_DEF SHUNT_VOLTAGE_CONT
//Calibration register: This value is needed to recalculate for each shunt
#define SHUNT_15mOhm_1mA_LSB 0x155

#define LED_OFF 0
#define LED_ON 1

#define DEFAULT_BLINK_PERIOD_MS 1000 
#define DEFAULT_BLINK_DUTY 128 // 255 - максимум
#define DEFAULT_BLINK_MODE LED_BLINK

typedef struct
{
	I2C_HandleTypeDef* i2c_ptr;
	uint8_t addr;
	uint16_t rx_data;
	uint8_t rx_reg_addr;
	uint8_t id_data[4];
	uint16_t voltage, current, power;
} type_INA226_DEVICE;

uint8_t ina226_init(type_INA226_DEVICE* ina226_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t addr);
uint16_t ina226_read_request(type_INA226_DEVICE* ina226_ptr, uint8_t reg_addr);
uint16_t ina226_read_IT_callback(type_INA226_DEVICE* ina226_ptr);

#endif
