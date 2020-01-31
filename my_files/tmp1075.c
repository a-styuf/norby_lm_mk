/**
  ******************************************************************************
  * @file           : tmp1075.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX для удобного управления термодатчиком TMP1075
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "tmp1075.h"

/**
  * @brief  инициализация работы с TMP1075: специально использую блокирующий режим
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  addr: адрес TMP1075
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint8_t tmp1075_init(type_TMP1075_DEVICE* tmp1075_ptr, I2C_HandleTypeDef* i2c_ptr, uint8_t addr)
{
	uint8_t i2c_rx_data[16] = {0}, validate_data[8] = {0};
	uint8_t i2c_tr_data[8] = {0};
	uint16_t i2c_reg_val = 0;
	//создаем структуру
	tmp1075_ptr->addr = addr;
	tmp1075_ptr->i2c_ptr = i2c_ptr;
	tmp1075_ptr->temp = 0;
	tmp1075_ptr->temp_high = HLIM_DEFAULT;
	tmp1075_ptr->temp_low = LLIM_DEFAULT;
	tmp1075_ptr->error_cnt = 0;
	memset(tmp1075_ptr->validate_data, 0x00, 8);
	// устанавливаем режим работы
	i2c_reg_val = CONFIG_DEFAULT;
	i2c_tr_data[0] = CFGR_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 3, 1000);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data, 2, 1000);
	// устанавливаем верхний уровень срабатывания alert
	i2c_reg_val = HLIM_DEFAULT;
	i2c_tr_data[0] = HLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+2, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 3, 1000);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+2, 2, 1000);
	// устанавливаем нижний уровень срабатывания alert
	i2c_reg_val = LLIM_DEFAULT;
	i2c_tr_data[0] = LLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+4, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 3, 1000);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+4, 2, 1000);	
	//
	i2c_tr_data[0] = DEV_ID_REGISTER_ADDR;
	i2c_reg_val = TMP_DEV_ID;
	memcpy(validate_data+6, (uint8_t*)&i2c_reg_val, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 1, 1000);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+6, 2, 1000);
	//
	memcpy(tmp1075_ptr->validate_data, i2c_rx_data, 8);
	// проверка на привильность записи
	if (memcmp((uint8_t*)i2c_rx_data, (uint8_t*)validate_data, 8) == 0){
		return 1;
	}
	return 0;
}

/**
  * @brief  установка уровней срабатывания сигнала Alert
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  temp_high: верхний порог температуры для срабатывания сигнала Alert в 1/256°С
  * @param  temp_low: нижний порог температуры для срабатывания сигнала Alert в 1/256°С
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint8_t tmp1075_alert_lvl_set(type_TMP1075_DEVICE* tmp1075_ptr, uint16_t temp_high,  uint16_t temp_low)
{
	uint8_t i2c_rx_data[16] = {0}, validate_data[8] = {0};
	uint8_t i2c_tr_data[8] = {0};
	uint16_t i2c_reg_val = 0;
	tmp1075_ptr->temp_high = temp_high;
	tmp1075_ptr->temp_low = temp_low;
	// устанавливаем верхний уровень срабатывания alert
	i2c_reg_val = tmp1075_ptr->temp_high;
	i2c_tr_data[0] = HLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+2, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_tr_data, 3, 100);
	HAL_I2C_Master_Receive(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_rx_data+2, 2, 100);
	// устанавливаем нижний уровень срабатывания alert
	i2c_reg_val = tmp1075_ptr->temp_low;
	i2c_tr_data[0] = LLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+4, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_tr_data, 3, 100);
	HAL_I2C_Master_Receive(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_rx_data+4, 2, 100);	
	return 0;
}


/**
  * @brief  установка адреса регистра TMP1075 для работы в режиме с прерыванием
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  reg_addr: адрес регистра для будующего чтения
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint8_t tmp1075_reg_addr_set(type_TMP1075_DEVICE* tmp1075_ptr, uint8_t reg_addr)
{
	
	tmp1075_ptr->tx_data[0]  = reg_addr;
	tmp1075_ptr->rx_reg_addr = reg_addr;
	HAL_I2C_Master_Transmit_IT(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, tmp1075_ptr->tx_data, 1);
	return 0;
}

/**
  * @brief  запрос данных из TMP1075 для работы в режиме с прерыванием
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint16_t tmp1075_read_request(type_TMP1075_DEVICE* tmp1075_ptr)
{
	HAL_I2C_Master_Receive_IT(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, (uint8_t*)&tmp1075_ptr->rx_data, 2);
	return 0;
}

/**
  * @brief  запрос данных из TMP1075 для работы в режиме с прерыванием
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint16_t tmp1075_read_data_process(type_TMP1075_DEVICE* tmp1075_ptr)
{
	switch(tmp1075_ptr->rx_reg_addr){
		case TEMP_REGISTER_ADDR:
			tmp1075_ptr->temp = __REV16(tmp1075_ptr->rx_data); //значение, где старший байт - вольты, младший - дробная часть в 1/256 В
			break;
		case LLIM_REGISTER_ADDR:
			tmp1075_ptr->temp_high = __REV16(tmp1075_ptr->rx_data); //значение, где старший байт - вольты, младший - дробная часть в 1/256 В
			break;
		case HLIM_REGISTER_ADDR:
			tmp1075_ptr->temp_low = __REV16(tmp1075_ptr->rx_data); //значение, где старший байт - вольты, младший - дробная часть в 1/256 В
			break;
	}
	return 0;
}

/**
  * @brief  обработка ошибок общения с перефеией I2C
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
void tmp1075_error_process(type_TMP1075_DEVICE* tmp1075_ptr)
{
	tmp1075_ptr->error_cnt += 1;
	tmp1075_ptr->queue_state = 0;
}

/**
  * @brief  старт чтения данных из TMP1075 c использованием очереди на прерываниях
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  */
void tmp1075_start_read_queue(type_TMP1075_DEVICE* tmp1075_ptr)
{
	tmp1075_ptr->queue_state = 1;
	tmp1075_reg_addr_set(tmp1075_ptr, TEMP_REGISTER_ADDR);
}

/**
  * @brief  развертывание очереди на прерываниях 
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  */
void tmp1075_body_read_queue(type_TMP1075_DEVICE* tmp1075_ptr)
{
		switch (tmp1075_ptr->queue_state){
			case 1:
				tmp1075_read_request(tmp1075_ptr);
				break;
			case 2:
				tmp1075_read_data_process(tmp1075_ptr);
				tmp1075_ptr->queue_state = 0;
				return;
			default:
				break;
		}
		tmp1075_ptr->queue_state += 1;
}
