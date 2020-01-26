/**
  ******************************************************************************
  * @file           : ina226.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX для удобного управления INA226 через I2C
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "ina226.h"

/**
  * @brief  проверка связи с INA226: специально использую блокирующий режим
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  addr: адрес INA226
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint8_t ina226_init(type_INA226_DEVICE* ina226_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t addr)
{
	uint8_t i2c_rx_data[16] = {0}, validate_data[8] = {0};
	uint8_t i2c_tr_data[8] = {0};
	uint16_t i2c_reg_val = 0;
	//создаем структуру
	ina226_ptr->addr = addr;
	ina226_ptr->i2c_ptr = i2c_ptr;
	ina226_ptr->voltage = 0;
	ina226_ptr->current = 0;
	ina226_ptr->power = 0;
	memset(ina226_ptr->id_data, 0x00, 4);
	// устанавливаем режим работы
	i2c_reg_val = (AVEREGES_DEF<<9) | (CONV_TIME_DEF<<6) | (CONV_TIME_DEF<<3) | (MODE_DEF<<0);
	i2c_tr_data[0] = CONFIG_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 3, 100);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data, 2, 100);
	// устанавливаем калибровки
	i2c_reg_val = SHUNT_15mOhm_1mA_LSB;
	i2c_tr_data[0] = CAL_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+2, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 2, 100);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+2, 2, 100);
	//
	i2c_tr_data[0] = MANUFACTURED_ID_REG_ADDR;
	i2c_reg_val = 0x5449;
	memcpy(validate_data+4, (uint8_t*)&i2c_reg_val, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 1, 100);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+4, 2, 100);
	//
	i2c_tr_data[0] = DIE_ID_REG_ADDR;
	i2c_reg_val = 0x2260;
	memcpy(validate_data+6, (uint8_t*)&i2c_reg_val, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 1, 100);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+6, 2, 100);
	//
	memcpy(ina226_ptr->id_data, i2c_rx_data+4, 4);
	// проверка на привильность записи
	if (memcmp((uint8_t*)i2c_rx_data, (uint8_t*)validate_data, 8) == 0){
		return 1;
	}
	return 0;
}

/**
  * @brief  проверка связи с INA226
  * @param  ina226_ptr: структура CubeMX для управления INA226
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint8_t ina226_check(type_INA226_DEVICE* ina226_ptr)
{
	uint8_t reg_addr = 0xFE;
	uint8_t i2c_rx_data[16] = {0};
	HAL_I2C_Master_Transmit(ina226_ptr->i2c_ptr, ina226_ptr->addr << 1, &reg_addr, 1, 100);
	HAL_I2C_Master_Receive(ina226_ptr->i2c_ptr, ina226_ptr->addr << 1, i2c_rx_data, 2, 100);
	if (i2c_rx_data[0] == 'T' && i2c_rx_data[1] == 'I')
	{
		return 1;
	}
	return 0;
}

/**
  * @brief  запрос данных из INA226 для работы в режиме с прерыванием
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  addr: адрес INA226
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint16_t ina226_read_request(type_INA226_DEVICE* ina226_ptr, uint8_t reg_addr)
{
	ina226_ptr->rx_reg_addr = reg_addr;
	HAL_I2C_Master_Transmit(ina226_ptr->i2c_ptr, ina226_ptr->addr << 1, &reg_addr, 1, 100);
	HAL_I2C_Master_Receive_IT(ina226_ptr->i2c_ptr, ina226_ptr->addr << 1, (uint8_t*)&ina226_ptr->rx_data, 2);
	return 0;
}


/**
  * @brief  запрос данных из INA226 для работы в режиме с прерыванием
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  addr: адрес INA226
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint16_t ina226_read_IT_callback(type_INA226_DEVICE* ina226_ptr)
{
	switch(ina226_ptr->rx_reg_addr){
		case VBUS_REGISTER_ADDR:
			ina226_ptr->voltage = ((1.25 * ina226_ptr->rx_data) / 1000.) * 256.; //значение, где старший байт - вольты, младший - дробная часть в 1/256 В
			break;
		case CURRENT_REGISTER_ADDR:
			ina226_ptr->current = (ina226_ptr->rx_data / 1000.) * 256.; //значение, где старший байт - вольты, младший - дробная часть в 1/256 В
			break;
		case POWER_REGISTER_ADDR:
			ina226_ptr->current = ((20 * ina226_ptr->rx_data) / 1000.) * 256.; //значение, где старший байт - вольты, младший - дробная часть в 1/256 В
			break;
	}
	return 0;
}


