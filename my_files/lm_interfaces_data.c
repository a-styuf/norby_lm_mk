/**
  ******************************************************************************
  * @file           : lm_interfaces_data.c
  * @version        : v1.0
  * @brief          : содержит структуры данных для работы с БРК, а также функции формирования кадра по 128-байт
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "lm_interfaces_data.h"

/**
  * @brief  инициализация структуры кадра из 128 байт
  * @param  header_ptr: указатель на структуру с заголовком
  * @param  dev_id: номер устройства
  * @param  type: тип кадра: 0-одиночный кадр, 1-заголовок архивного кадра, 2-тело архивного кадра
  * @param  d_code: тип данных (согласно принятому соглашению: например "Маяк" или "ТМИ")
  * @param  num: если тип кадра одиночный, то не используется, если архивный:
  *                   - одиночный: не проверяется
  *                   - заголовок архива: количество кадров
  *                   - тело архива: номер кадра архва (нулевой - это заголовок, соотетственно первый кадр тела - 1й)
  * @param  time_s: время в секундах по часам устройства, формирующего кадр
  * @retval длина получившегося заголовка (для разных типов данных длина разная), 0 - ошибка
  */
uint8_t frame_create_header(uint8_t* header_ptr, uint8_t dev_id, uint8_t type, uint8_t d_code, uint16_t* fr_num, uint16_t num, uint32_t time_s)
{
	type_SingleFrame_Header* s_header;
	type_ArchHeadFrame_Header* ah_header;
	type_ArchHeadFrame_Header* ab_header;
  switch(type){
    case SINGLE_FRAME_TYPE:
      s_header = (type_SingleFrame_Header*)header_ptr;
      s_header->mark = FRAME_MARK;
      s_header->id_loc.fields.dev_id = dev_id & 0xF;
      s_header->id_loc.fields.flags = 0x0 & 0xF;
      s_header->id_loc.fields.data_code = d_code & 0xFF;
      s_header->num = *fr_num;
      s_header->time = time_s;
			*fr_num += 1;
      return sizeof(type_SingleFrame_Header);
    case ARCH_HEADER_FRAME_TYPE:
      ah_header = (type_ArchHeadFrame_Header*)header_ptr;
      ah_header->mark = FRAME_MARK;
      ah_header->id_loc.fields.dev_id = dev_id & 0xF;
      ah_header->id_loc.fields.flags = 0x0 & 0xF;
      ah_header->id_loc.fields.data_code = d_code & 0xFF;
      ah_header->num = *fr_num;
      ah_header->time = time_s;
      ah_header->arch_len = num;
			*fr_num += 1;
      return sizeof(type_ArchHeadFrame_Header);
    case ARCH_BODY_FRAME_TYPE:
      ab_header = (type_ArchHeadFrame_Header*)header_ptr;
      ab_header->mark = FRAME_MARK;
      ab_header->id_loc.fields.dev_id = dev_id & 0xF;
      ab_header->id_loc.fields.flags = (0x1 << 0) & 0xF;
      ab_header->id_loc.fields.data_code = d_code & 0xFF;
      ab_header->num = *fr_num;
      ab_header->time = time_s;
      ab_header->arch_len = num;
			*fr_num += 1;
      return sizeof(type_ArchHeadFrame_Header);
  }
  return 0;
}

/**
  * @brief  прлсчеи и подстановка crc16 в гот овый кадр из 128 байт
  * @param  frame_ptr: указатель на кадр
  */
void frame_crc16_calc(uint8_t* frame_ptr)
{
  uint16_t crc16;
  crc16 = norby_crc16_calc(frame_ptr, 126);
  frame_ptr[126] = (crc16 >> 8) & 0xFF;
  frame_ptr[127] = (crc16 >> 0) & 0xFF;
}
