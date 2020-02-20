/**
  ******************************************************************************
  * @file           : pn_11_interface_tr_lvl.c
  * @version        : v1.0
  * @brief          : реализация транспортного уровня протокола общения с ПН1.1. Канальный уровень реализуется в HAL СubeMX.
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "pn_11_interface_tr_lvl.h"

/**
  * @brief  инициализация транспортного уровня протокола
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  huart: указатель на структуру UART
  */
void tr_lvl_init(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, UART_HandleTypeDef* huart)
{
	memset(tr_lvl_ptr, 0x00, sizeof(type_PN11_INTERFACE_TR_LVL));
	tr_lvl_ptr->huart = huart;
}

/**
  * @brief  запуск
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  data: указатель на массив данных для передачи
  * @param  len: длинна данных для передачи
  */
uint8_t tr_lvl_send_data(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t* data, uint8_t len)
{
	//формируем и запрашиваем место в буфере
	tx_create_frame(tr_lvl_ptr, FR_SPACE_REQ, NULL, NULL);
	tx_uart_data(tr_lvl_ptr);
	//записываем в структуру данные на отправку
	memcpy(tr_lvl_ptr->row_tx_data, data, len);
	tr_lvl_ptr->row_tx_len = len;
	return 1;
}

/**
  * @brief  запуск
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  data: указатель на массив данных для передачи
  * @param  len: длинна данных для передачи
  */
void tr_lvl_process_10ms(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr)
{
	int8_t rx_status = 0;
	if (tr_lvl_ptr->rx_len) {
		rx_status = rx_check_frame(tr_lvl_ptr);
		if (rx_status == NO_RECOGNISED_FRAME){ //недостаточно данных для распознавания
			return;
		}
		else if (rx_status >= 0){ //удачный прием данных
			switch(rx_status){
				case FR_SPACE_REQ:
					tx_create_frame(tr_lvl_ptr, FR_SPACE_ANS, NULL, NULL);
					break;
				case FR_SPACE_ANS:
					if (tr_lvl_ptr->tx_space >= tr_lvl_ptr->tx_len){  // отправляем данные
						tx_create_frame(tr_lvl_ptr, FR_DATA, tr_lvl_ptr->row_tx_data, tr_lvl_ptr->row_tx_len);
						tr_lvl_ptr->tx_space = 0;
					}
					else{  // если места для данных не хватает, то запршивавем место заново
						tx_create_frame(tr_lvl_ptr, FR_SPACE_REQ, NULL, NULL);
					}
					break;
				case FR_LAST_STATUS_REQ:
					tx_create_frame(tr_lvl_ptr, FR_LAST_STATUS_ANS, NULL, NULL);
					break;
				case FR_LAST_STATUS_ANS:
					switch (tr_lvl_ptr->tx_error_code){
						case ERR_TYPE_OK:
							tr_lvl_ptr->tx_state = TX_OK;
							tr_lvl_ptr->tx_len = 0;
							break;
						case ERR_TYPE_CRC: // повторяем отправку данных справильным номером данных
							if(tr_lvl_ptr->tx_last_sended_type == FRAME){
								tr_lvl_ptr->tx_state = CRC_HEADER_ERROR;
							}
							else{
								tr_lvl_ptr->tx_state = CRC_DATA_ERROR;
							}
							break;
						case ERR_TYPE_SPACE:
							tx_create_frame(tr_lvl_ptr, FR_SPACE_ANS, NULL, NULL);
							tr_lvl_ptr->tx_state = SPACE_ERROR;
							break;
						case ERR_TYPE_NUM: // повторяем отправку данных справильным номером данных
							tr_lvl_ptr->tx_data_frame_num = tr_lvl_ptr->tx_error_frame_num;
							tx_create_frame(tr_lvl_ptr, FR_DATA, tr_lvl_ptr->row_tx_data, tr_lvl_ptr->row_tx_len);
							tr_lvl_ptr->tx_state = NUME_ERROR;
							break;
					}
					break;
				case FR_RST_REQ:
					NULL;
					break;
				case FR_DATA:
					tx_create_frame(tr_lvl_ptr, FR_LAST_STATUS_ANS, NULL, NULL);
					break;
			}
			if (tr_lvl_ptr->tx_len){ 
				tx_uart_data(tr_lvl_ptr);
			}
		}
		else if (rx_status < 0){ //неудачный прием данных
			switch(rx_status){
				case -3:
					tx_create_frame(tr_lvl_ptr, FR_LAST_STATUS_ANS, NULL, NULL);
					break;
			}
		}
	}
	else{
		return;
	}
}

/**
  * @brief  создание пакета для отправки
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  fr_type: тип формируемого пакета
  * @param  data: указатель на массив данных для передачи
  * @param  len: длинна данных для передачи
  */
void tx_create_frame(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t fr_type, uint8_t* data, uint8_t len)
{
	uint8_t tr_len = 0;
	tr_lvl_ptr->tx_frame_type = fr_type & 0x07;
	tr_lvl_ptr->tx_len = 4;
	tr_lvl_ptr->tx_data[0] = 0x55;
	tr_lvl_ptr->tx_data[1] = (tr_lvl_ptr->tx_frame_type << 5) | (tr_lvl_ptr->tx_data_frame_num & 0x1F);
	switch (tr_lvl_ptr->tx_frame_type){
		case FR_SPACE_REQ:
		case FR_RST_REQ:
		case FR_LAST_STATUS_REQ:
			tr_lvl_ptr->tx_data[2] = 0x00;
			break;
		case FR_SPACE_ANS:
			tr_lvl_ptr->tx_data[2] = DT_MAX_LEN - tr_lvl_ptr->rx_len;
			break;
		case FR_DATA:
			if ((len) > DT_MAX_LEN) tr_len = DT_MAX_LEN;
			else tr_len = len;
			tr_lvl_ptr->tx_data[2] = tr_len;
			tx_create_data_frame(tr_lvl_ptr, data, tr_len);
			tr_lvl_ptr->tx_len += tr_len + 1;
			tr_lvl_ptr->tx_data_frame_num += 1;
			break;
		case FR_LAST_STATUS_ANS:
			tr_lvl_ptr->tx_data[2] = tx_get_error_type(tr_lvl_ptr);
			break;
	}
	tr_lvl_ptr->tx_data[3] = crc8_rmap_header(tr_lvl_ptr->tx_data, 3);
	
}

/**
  * @brief  создание пакета данных
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  data: указатель на массив данных для передачи
  * @param  len: длинна данных для передачи
  */
void tx_create_data_frame(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t* data, uint8_t len)
{
	memcpy(&tr_lvl_ptr->tx_data[4], data, len); 
	tr_lvl_ptr->tx_data[4+len]  = crc8_rmap_data(&tr_lvl_ptr->tx_data[4], len);
}

/**
  * @brief  проверка и выдача ошибки
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  data: указатель на массив данных для передачи
  * @param  len: длинна данных для передачи
  */
uint8_t tx_get_error_type(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr)
{
	if (tr_lvl_ptr->rx_error_flag != 0){
		tr_lvl_ptr->rx_error_flag  = 0;
		return tr_lvl_ptr->rx_error_type;
	}
	else{
		tr_lvl_ptr->rx_error_type = ERR_TYPE_OK;
	}
	return 0;
}

/**
  * @brief  распознование пришедшего пакета
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @retval статус ошибки: 0x - нет пакета, >0 - тип пакета, <0 код ошибки
  */
int8_t rx_check_frame(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr)
{
	uint8_t data_state = 0;
	int8_t status;
	if(tr_lvl_ptr->rx_len < 4){  //пакет не может быть меньше 4-х байт
		status = NO_RECOGNISED_FRAME;
	}
	else{
		if (tr_lvl_ptr->rx_data[0] != 0x55){ //пакет обязательно начинается с 0x55
			memmove(tr_lvl_ptr->rx_data, tr_lvl_ptr->rx_data+1, tr_lvl_ptr->rx_len);
			tr_lvl_ptr->rx_len -= 1;
			rx_error_set(tr_lvl_ptr, ERR_TYPE_OTHERS);
			status = NO_RECOGNISED_FRAME;
		}
		else{
			tr_lvl_ptr->rx_frame_type = (tr_lvl_ptr->rx_data[1] >> 5) & 0x07;
			if ((crc8_rmap_header(tr_lvl_ptr->rx_data, 3) == tr_lvl_ptr->rx_data[3])){
				switch (tr_lvl_ptr->rx_frame_type){ // определяем статус пакета
					case FR_SPACE_REQ:
						status = FR_SPACE_REQ;
						break;
					case FR_SPACE_ANS:
						tr_lvl_ptr->tx_space = tr_lvl_ptr->rx_data[1];
						status = FR_SPACE_ANS;
						break;
					case FR_LAST_STATUS_REQ:
						status = FR_LAST_STATUS_REQ;
						break;
					case FR_LAST_STATUS_ANS:
						rx_error_check(tr_lvl_ptr, tr_lvl_ptr->rx_data[2]);
						status = FR_LAST_STATUS_ANS;
						break;
					case FR_RST_REQ:
						NULL;
						status = FR_RST_REQ;
						break;
					case FR_DATA:
						data_state = rx_data_check(tr_lvl_ptr);
						switch (data_state){
							case NO_RECOGNISED_FRAME:
								status = NO_RECOGNISED_FRAME;
							break;
							case INCORRECT_DATA_CRC8:
								rx_error_set(tr_lvl_ptr, ERR_TYPE_DATA_CRC);
								status = FR_DATA;							
							break;
							case INCORRECT_DATA_NUM:
								rx_error_set(tr_lvl_ptr, ERR_TYPE_NUM);
								status = FR_DATA;							
							break;
							case CORRECT_DATA:
								tr_lvl_ptr->rx_data_frame_num += 1;
								status = FR_DATA;							
							break;
						}
						break;
					default:
						status = -2; // нераспознан статус пакета
						break;
				}
			}
			else{
				rx_error_set(tr_lvl_ptr, ERR_TYPE_HEADER_CRC);
				status = -4; // некорректный crc
			}
			
		}
	}
	if (status != NO_RECOGNISED_FRAME) tr_lvl_ptr->rx_len = 0;
	return status;
}

/**
  * @brief  формирование ошибки
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  data: указатель на массив данных для передачи
  * @param  len: длинна данных для передачи
  */
void rx_error_set(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t type)
{
	tr_lvl_ptr->rx_error_flag = 0;
	switch(type){
		case ERR_TYPE_OK:
		case ERR_TYPE_HEADER_CRC:
		case ERR_TYPE_SPACE:
		case ERR_TYPE_NUM:
			break;
		case ERR_TYPE_DATA_CRC:
			tr_lvl_ptr->rx_error_flag |= 0x10;
			break;
		default:
			tr_lvl_ptr->rx_error_type = 0xFF;
			break;
	}
	//
	tr_lvl_ptr->rx_error_type = ((type & 0x07) << 5) | (tr_lvl_ptr->rx_data_frame_num & 0x1F);
	tr_lvl_ptr->rx_error_code = ((type & 0x07) << 5);
	tr_lvl_ptr->rx_error_frame_num = (tr_lvl_ptr->rx_data_frame_num & 0x1F);
	//
	if(type != ERR_TYPE_OK) {
		tr_lvl_ptr->rx_error_cnt += 1;
		tr_lvl_ptr->rx_error_flag |= 0x01;
	}
}

/**
  * @brief  проверяем тип ошибки, который был прислан; перекладываем её в состояние отправки
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @param  data: указатель на массив данных для передачи
  */
void rx_error_check(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr, uint8_t err_type)
{
	tr_lvl_ptr->tx_error_type = err_type;
	tr_lvl_ptr->tx_error_code = (err_type >> 5) & 0x07;
	tr_lvl_ptr->tx_error_frame_num = (err_type >> 0) & 0x1F;
	if (tr_lvl_ptr->tx_error_code != ERR_TYPE_OK){
		tr_lvl_ptr->tx_error_cnt += 1;
		tr_lvl_ptr->tx_error_flag = 1;
	}
}

/**
  * @brief  проверка пришедших данных на правильность
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  * @retval статус ошибки: согласно хэдэру 
  */
uint8_t rx_data_check(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr)
{
	uint8_t data_len=0, frame_len=0; 
	//
	data_len = tr_lvl_ptr->rx_data[2];
	frame_len = 4+data_len+1;
	//
	if (frame_len > tr_lvl_ptr->rx_len){
		return NO_RECOGNISED_FRAME;
	}
	else if(tr_lvl_ptr->rx_data_frame_num != ((tr_lvl_ptr->rx_data[1] >> 0) & 0x1F)) {
		return INCORRECT_DATA_NUM;
	}
	else if(tr_lvl_ptr->rx_data[4+data_len] != crc8_rmap_data(&tr_lvl_ptr->rx_data[4], data_len)){
		return INCORRECT_DATA_CRC8;
	}
	else{
		tr_lvl_ptr->row_rx_len = data_len;
		memcpy(tr_lvl_ptr->row_rx_data, &tr_lvl_ptr->rx_data[4], data_len);
		return CORRECT_DATA;
	}	
}

/**
  * @brief  надстройка над отправкой по UART
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  */
void tx_uart_data(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr)
{
	HAL_UART_Transmit_IT(tr_lvl_ptr->huart, tr_lvl_ptr->tx_data, tr_lvl_ptr->tx_len);
	//tr_lvl_ptr->tx_len = 0;
}

/**
  * @brief  надстройка над  по UART
  * @param  tr_lvl_ptr: указатель на структуру управления транспортным уровнем
  */
void rx_uart_data(type_PN11_INTERFACE_TR_LVL* tr_lvl_ptr)
{
	tr_lvl_ptr->rx_len++;
	HAL_UART_Receive_IT(tr_lvl_ptr->huart, &tr_lvl_ptr->rx_data[tr_lvl_ptr->rx_len], 1);
}
