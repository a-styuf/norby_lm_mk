/**
  ******************************************************************************
  * @file           : modbus.c
  * @version        : v1.0
  * @brief          : библиотека для работы с modbus через HAL_Uart
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "modbus.h"

/**
  * @brief  инициализация програмного модуля для работы с MODBUS
  * @param  mb_ptr: указатель на структуру управления modbus
	* @param  uart_ptr: указатель на hal-uart, используемый для данной ПН
  */
void mb_init(type_MODBUS* mb_ptr, UART_HandleTypeDef *uart_ptr)
{
	// привязываем конкретный uart к полезной нагрузке
	mb_ptr->huart = uart_ptr;
	// обнуляем переменные состояний
	mb_ptr->tx_len = 0;
	mb_ptr->rx_len = 0;
	mb_ptr->row_rx_len = 0;
	mb_ptr->rx_ptr = 0;
	mb_ptr->rx_ptr_offset = 0;
	mb_ptr->rx_timeout_flag = 0;
	mb_ptr->rx_timeout = 0;
	mb_ptr->rx_data_ready = 0;
	// чистим буферы приема и структуры кадров на всякий случай
	memset(mb_ptr->tx_data, 0x00, sizeof(mb_ptr->tx_data));
	memset(mb_ptr->rx_data, 0x00, sizeof(mb_ptr->rx_data));
	memset(mb_ptr->row_rx_data, 0x00, sizeof(mb_ptr->row_rx_data));
}

/**
  * @brief  обработка и прием данных modbus
  * @param  mb_ptr указатель на структуру управления modbus
	* @param  period_ms задержка вызова функции
  */
void mb_process(type_MODBUS* mb_ptr, uint16_t period_ms)
{
	// обработка таймаута
	mb_rx_timeout_cb(mb_ptr, period_ms);
	// анализ принятых данных	
}

/**
  * @brief  сбрс указателя rx_ptr c перезапуском чтения
  * @param  mb_ptr: указатель на структуру управления
  */
void mb_rx_ptr_reset(type_MODBUS* mb_ptr)
{
	mb_ptr->rx_ptr = 0;
	mb_ptr->rx_ptr_offset = 0;
	HAL_UART_AbortReceive_IT(mb_ptr->huart);
	HAL_UART_Receive_IT(mb_ptr->huart, mb_ptr->rx_data + mb_ptr->rx_ptr, 1);
}

/**
  * @brief  мгновенная отправка команды c произвольным кодом
  * @param  mb_ptr указатель на структуру управления
	* @param  dev_id адрес утсройства
	* @param  f_code код команды
	* @param  reg_addr адрес начального регистра для передачи
	* @param  reg_cnt количество регистров для передачи (проверяется только для команд с передачей/чтением нескольких регистров)
	* @retval  статус: >= 0 - OK, < 0 - ошибка
  */
int8_t mb_send_frame(type_MODBUS* mb_ptr, uint8_t dev_id, uint8_t f_code, uint16_t reg_addr, uint16_t reg_cnt, uint16_t* data)
{
	mb_ptr->tx_frame.type = MB_FRAME_TYPE_TX;
	mb_ptr->tx_frame.dev_id = dev_id;
	mb_ptr->tx_frame.f_code = f_code;
	mb_ptr->tx_frame.reg_addr = reg_addr;
	mb_ptr->tx_frame.reg_cnt = reg_cnt;
	mb_ptr->tx_frame.byte_cnt = reg_cnt * 2;
	if ((f_code == 6) || (f_code == 16)){
		memcpy((uint8_t*)mb_ptr->tx_frame.data, (uint8_t*)data, reg_cnt*2);
	}
	mb_frame_calc_crc16(&mb_ptr->tx_frame);
	mb_frame_form_packet(&mb_ptr->tx_frame, mb_ptr->tx_data, &mb_ptr->tx_len);
	//отправляем данные
	HAL_UART_Transmit_IT(mb_ptr->huart, mb_ptr->tx_data, mb_ptr->tx_len);
	return 0;
}

/**
  * @brief  выборка пакета MB из входящего потока данных
  * @param  mb_ptr указатель на структуру управления
	* @retval  1 - пакет принят, 0 - отсутсвие пакета, -1 = потрянный пакет
  */
int8_t mb_get_frame(type_MODBUS* mb_ptr)
{
		//todo
}


/**
  * @brief  обработка колбэка на прием данных (для вызова в CB от прерывания на прием)
  * @param  mb_ptr указатель на структуру управления
  */
void mb_int_rx_huart_cb(type_MODBUS* mb_ptr)
{
	// инкрементация длины
	mb_ptr->rx_ptr++;
	// запрос нового байта
	HAL_UART_Receive_IT(mb_ptr->huart, mb_ptr->rx_data + mb_ptr->rx_ptr, 1);
}

/**
  * @brief  обаработка таймаута на прием данных
  * @param  mb_ptr указатель на структуру управления
	* @param  period_ms значение периода вызова данной функции
  */
void mb_rx_timeout_cb(type_MODBUS* mb_ptr, uint16_t period_ms)
{
	if (mb_ptr->rx_timeout_flag == 1){
		if ((mb_ptr->rx_timeout == 0) || (mb_ptr->rx_timeout > (0xFFFFFFFF - period_ms))){
			mb_ptr->rx_timeout = 0;
		}
		else{
			mb_ptr->rx_timeout -= period_ms;
		}
	}
	else{
		//
	}
}

/**
  * @brief  проверка массива на наличие в нем пакета ModBus
  * @param  mb_ptr указатель на структуру управления
	* @param  data указатель на массив данных для проверки
	* @param  len длина массива данных для проверки
	* @retval  статус
  */
int8_t mb_int_check_frame(type_MODBUS* mb_ptr, uint8_t* data, uint8_t len)
{
	uint8_t dev_id, f_code = 0, b_cnter = 0, frame_length = 0;
	uint8_t error_code;
	uint16_t reg_addr, reg_cnter;
	uint16_t mb_data[128] = {0};
	if (len < 5){ // если меньше 5-ти байт (минимальная длина кадра для ModBus), то ждем еще данных
		return MB_UART_RX_STATUS_SHORT;
	}
	else{
		dev_id = data[0];
		f_code = data[1];
		if (f_code & 0x80){
			f_code &= 0x7F;
			error_code = data[2];
			frame_length = 5;
			if (__mb_crc16(data, frame_length, MB_CRC16_INIT_VAL) == 0){
				return MB_UART_RX_STATUS_OK;
			}
			else{
				return MB_UART_CRC_ERROR;
			}
		}
		else{
			switch(f_code){
				case 3:
					b_cnter = data[2];
					frame_length = b_cnter + 5;
					if (__mb_crc16(data, frame_length, MB_CRC16_INIT_VAL) == 0){
						return MB_UART_RX_STATUS_OK;
					}
					else{
						return MB_UART_CRC_ERROR;
					}
				case 6:
					reg_addr = *((uint16_t*)&data[2]);
					memcpy(&mb_data, &data[4], 2);
					frame_length = 8;
					if (__mb_crc16(data, frame_length, MB_CRC16_INIT_VAL) == 0){
						return MB_UART_RX_STATUS_OK;
					}
					else{
						return MB_UART_CRC_ERROR;
					}
				case 10:
					reg_addr = *((uint16_t*)&data[2]);
					reg_cnter = *((uint16_t*)&data[4]);
					frame_length = 8;
					if (__mb_crc16(data, frame_length, MB_CRC16_INIT_VAL) == 0){
						return MB_UART_RX_STATUS_OK;
					}
					else{
						return MB_UART_CRC_ERROR;
					}
			}
		}
	}
	return 0;
}

/* --------------------------- work with frames ---------------------------------- */
/**
  * @brief  инициализация переменной типа MB-frame
  * @param  frame_ptr указатель на структуру кадра
  */
void mb_frame_init(type_MB_Frame* frame_ptr)
{
	memset((uint8_t*)frame_ptr, 0x00, sizeof(type_MB_Frame));
}

/**
  * @brief  подсчет контрольной суммы для кадра с учетом функционального кода
  * @param  frame_ptr указатель на структуру кадра
	* @retval crc16
  */
uint16_t mb_frame_calc_crc16(type_MB_Frame* frame_ptr)
{
	uint16_t crc16 = MB_CRC16_INIT_VAL;
	crc16 = __mb_crc16(&frame_ptr->dev_id, 1, crc16);
	crc16 = __mb_crc16(&frame_ptr->f_code, 1, crc16);
	if (frame_ptr->type == MB_FRAME_TYPE_RX) {
		switch(frame_ptr->f_code){
			case 3:
				crc16 = __mb_crc16(&frame_ptr->byte_cnt, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->data, frame_ptr->byte_cnt, crc16);
				break;
			case 6:
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 0, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->data[0], 2, crc16);
				break;
			case 16:
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 0, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_cnt + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_cnt + 0, 1, crc16);
				break;
		}
	}
	else if (frame_ptr->type == MB_FRAME_TYPE_TX) {
		switch(frame_ptr->f_code){
			case 3:
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 0, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_cnt + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_cnt + 0, 1, crc16);
				break;
			case 6:
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 0, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->data[0], 2, crc16);
				break;
			case 16:
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_addr + 0, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_cnt + 1, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->reg_cnt + 0, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->byte_cnt, 1, crc16);
				crc16 = __mb_crc16((uint8_t*)frame_ptr->data, frame_ptr->byte_cnt, crc16);
				break;
		}
	}
	else if (frame_ptr->type == MB_FRAME_TYPE_ERROR) {
		switch(frame_ptr->f_code & 0x7F){
			case 3:
				crc16 = __mb_crc16((uint8_t*)&frame_ptr->error_code, 1, crc16);
				break;
		}
	}
	frame_ptr->crc_16 = crc16;
	return crc16;
}

/**
  * @brief  формирование пакета для отправки
  * @param  frame_ptr указатель на структуру кадра
  */
void mb_frame_form_packet(type_MB_Frame* frame_ptr, uint8_t* data, uint8_t* leng)
{
	uint8_t l = 0, i = 0;
	data[0] = frame_ptr->dev_id;
	data[1] = frame_ptr->f_code;
	if (frame_ptr->type == MB_FRAME_TYPE_TX) {
		switch(frame_ptr->f_code){
			case 3:
				data[2] = (frame_ptr->reg_addr >> 8) & 0xFF;
				data[3] = (frame_ptr->reg_addr >> 0) & 0xFF;
				data[4] = (frame_ptr->reg_cnt >> 8) & 0xFF;
				data[5] = (frame_ptr->reg_cnt >> 0) & 0xFF;
				l = 6;
				break;
			case 6:
				data[2] = (frame_ptr->reg_addr >> 8) & 0xFF;
				data[3] = (frame_ptr->reg_addr >> 0) & 0xFF;
				data[4] = frame_ptr->data[0];
				data[5] = frame_ptr->data[1];
				l = 6;
				break;
			case 16:
				data[2] = (frame_ptr->reg_addr >> 8) & 0xFF;
				data[3] = (frame_ptr->reg_addr >> 0) & 0xFF;
				data[4] = (frame_ptr->reg_cnt >> 8) & 0xFF;
				data[5] = (frame_ptr->reg_cnt >> 0) & 0xFF;
				data[6] = (frame_ptr->byte_cnt >> 0) & 0xFF;
				l = 6;
				for (i = 0; i<frame_ptr->byte_cnt; i++){
					data[3+i] = frame_ptr->data[i];
					l += 1;
				}
				break;
		}
	}
	else if (frame_ptr->type == MB_FRAME_TYPE_RX) {
		switch(frame_ptr->f_code){
			case 3:
				data[2] = (frame_ptr->byte_cnt) & 0xFF;
				l = 3;
				for (i = 0; i<frame_ptr->byte_cnt; i++){
					data[3+i] = frame_ptr->data[i];
					l += 1;
				}
				break;
			case 6:
				data[2] = (frame_ptr->reg_addr >> 8) & 0xFF;
				data[3] = (frame_ptr->reg_addr >> 0) & 0xFF;
				data[4] = frame_ptr->data[0];
				data[5] = frame_ptr->data[1];
				l = 6;
				break;
			case 16:
				data[2] = (frame_ptr->reg_addr >> 8) & 0xFF;
				data[3] = (frame_ptr->reg_addr >> 0) & 0xFF;
				data[4] = (frame_ptr->reg_cnt >> 8) & 0xFF;
				data[5] = (frame_ptr->reg_cnt >> 0) & 0xFF;
				l = 6;
				break;
		}
	}
	else if (frame_ptr->type == MB_FRAME_TYPE_ERROR) {
		switch(frame_ptr->f_code & 0x7F){
			case 3:
				data[1] |= (0x80);
				data[2] = frame_ptr->error_code;
				l = 3;
				break;
		}
	}
	data[l] = (frame_ptr->crc_16 >> 0) & 0xFF;
	data[l+1] = (frame_ptr->crc_16 >> 8) & 0xFF;
	l += 2;
	*leng = l;
}

/* --------------------------- CRC16 ---------------------------------- */

const uint16_t _crc16_table[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
		};


uint16_t __mb_crc16(uint8_t *data, uint16_t len, uint16_t crc16_init) 
{
  uint16_t i = 0; /* will index into CRC lookup table */
	uint16_t crc16 = crc16_init;
	for (i = 0; i < len; i++) {
		crc16 = (crc16 >> 8) ^ _crc16_table[(crc16 ^ data[i]) & 0xFF];
		}
	return crc16;
}
