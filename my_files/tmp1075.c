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
	uint8_t i2c_rx_data[16] = {0}, validate_data[16] = {0};
	uint8_t i2c_tr_data[8] = {0};
	uint16_t i2c_reg_val = 0;
	//создаем структуру
	tmp1075_ptr->addr = addr;
	tmp1075_ptr->i2c_ptr = i2c_ptr;
	tmp1075_ptr->temp = 0;
	tmp1075_ptr->temp_high = ERROR_TEMP_HIGH;
	tmp1075_ptr->temp_low = ERROR_TEMP_LOW;
	tmp1075_ptr->temp_hyst = ERROR_TEMP_HYST;
	tmp1075_ptr->temp_high_hyst_state = 0;
	tmp1075_ptr->temp_low_hyst_state = 0;
	tmp1075_ptr->error = 0;
	memset(tmp1075_ptr->validate_data, 0x00, 16);
	// устанавливаем режим работы
	i2c_reg_val = CONFIG_DEFAULT;
	i2c_tr_data[0] = CFGR_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 3, 1000);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data, 2, 1000);
	// устанавливаем верхний уровень срабатывания alert
	i2c_reg_val = ALARM_HLIM_DEFAULT;
	i2c_tr_data[0] = HLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+2, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(i2c_ptr, addr << 1, i2c_tr_data, 3, 1000);
	HAL_I2C_Master_Receive(i2c_ptr, addr << 1, i2c_rx_data+2, 2, 1000);
	// устанавливаем нижний уровень срабатывания alert
	i2c_reg_val = ALARM_LLIM_DEFAULT;
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
	memcpy(tmp1075_ptr->validate_data, i2c_rx_data, 16);
	// проверка на привильность записи
	if (memcmp((uint8_t*)i2c_rx_data, (uint8_t*)validate_data, 16) == 0){
		return 1;
	}
	return 0;
}

/**
  * @brief  установка уровней срабатывания сигнала Alert (режим с компаратором работает только для превышения врехнего порога)
  * @param  i2c_ptr: структура CubeMX для управления ядром I2C
  * @param  temp_high: верхний порог температуры для срабатывания сигнала Alert в 1/256°С
  * @param  temp_low: нижний порог температуры для отпускания сигнала Alert в 1/256°С
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
uint8_t tmp1075_alert_lvl_set(type_TMP1075_DEVICE* tmp1075_ptr, int16_t temp_high,  int16_t temp_low)
{
	uint8_t i2c_rx_data[16] = {0}, validate_data[8] = {0};
	uint8_t i2c_tr_data[8] = {0};
	uint16_t i2c_reg_val = 0;
	// устанавливаем верхний уровень срабатывания alert
	i2c_reg_val = (uint16_t)tmp1075_ptr->temp_high;
	i2c_tr_data[0] = HLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+2, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_tr_data, 3, 100);
	HAL_I2C_Master_Receive(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_rx_data+2, 2, 100);
	// устанавливаем нижний уровень срабатывания alert
	i2c_reg_val = (uint16_t)tmp1075_ptr->temp_low;
	i2c_tr_data[0] = LLIM_REGISTER_ADDR;
	i2c_tr_data[1] = (i2c_reg_val>>8) & 0xFF;
	i2c_tr_data[2] = (i2c_reg_val>>0) & 0xFF;
	memcpy(validate_data+4, i2c_tr_data+1, 2);
	HAL_I2C_Master_Transmit(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_tr_data, 3, 100);
	HAL_I2C_Master_Receive(tmp1075_ptr->i2c_ptr, tmp1075_ptr->addr << 1, i2c_rx_data+4, 2, 100);	
	return 0;
}

/**
  * @brief  установка границ допустимых значений температуры для проверки в ПО
  * @param  tmp1075_ptr: структура управления каналом измерения температуры
  * @param  temp_high: верхняя граница допустимой температуры
  * @param  temp_low: нижняя граница допустимой температуры
  * @param  temp_hyst: гистерезис срабатывания ошибки по температуре
  */
void tmp1075_set_bound(type_TMP1075_DEVICE* tmp1075_ptr, int16_t temp_high,  int16_t temp_low, int16_t temp_hyst)
{
	tmp1075_ptr->temp_high = temp_high;
	tmp1075_ptr->temp_low = temp_low;
	tmp1075_ptr->temp_hyst = temp_hyst;
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
  */
void tmp1075_error_process(type_TMP1075_DEVICE* tmp1075_ptr)
{
	tmp1075_ptr->error |= TMP_CH_ERR_SENSOR;
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


/**
  * @brief  проверка параметров температуры
  * @param  tmp1075_ptr: структура управления каналом питания
  * @param  error: тип ошибки температуры
  * @retval 1 - изменилось состояние температуры, 0 - ошибки остались, какие и были
  */
uint8_t tmp1075_get_error(type_TMP1075_DEVICE* tmp1075_ptr, uint8_t *error)
{
	uint8_t report = 0, retval = 0;
	int16_t h_lim = 0, l_lim = 0;
	//
	h_lim = tmp1075_ptr->temp_high - (tmp1075_ptr->temp_high_hyst_state)*tmp1075_ptr->temp_hyst;
	l_lim = tmp1075_ptr->temp_low + (tmp1075_ptr->temp_low_hyst_state)*tmp1075_ptr->temp_hyst;
	if (tmp1075_ptr->temp >= h_lim){
		tmp1075_ptr->temp_high_hyst_state = 1;
		report|= TMP_CH_ERR_TOO_HIGH;
	}
	else{
		tmp1075_ptr->temp_high_hyst_state = 0;
	}
	if (tmp1075_ptr->temp < l_lim){
		tmp1075_ptr->temp_low_hyst_state = 1;
		report|=  TMP_CH_ERR_TOO_LOW;
	}
	else{
		tmp1075_ptr->temp_low_hyst_state = 0;
	}
	 //для определения изменения значения с нуля на 1 используется для old и new: (old^new)&new
	if ((((tmp1075_ptr->error ^ report) & report)) & 0x0F){
		retval = 1;
	}
	tmp1075_ptr->error = report;
	*error = report;
	return retval;
}
