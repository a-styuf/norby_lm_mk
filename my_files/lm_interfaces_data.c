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
  * @retval длина получившегося заголовка (для разных типов данных длина разная), 0 - ошибка
  */
uint8_t frame_create_header(uint8_t* header_ptr, uint8_t dev_id, uint8_t type, uint8_t d_code, uint16_t fr_num, uint16_t num)
{
	type_SingleFrame_Header* s_header;
	type_ArchHeadFrame_Header* ah_header;
	type_ArchBodyFrame_Header* ab_header;
	type_DCRFrame_Header* dcr_header;
  switch(type){
    case SINGLE_FRAME_TYPE:
      s_header = (type_SingleFrame_Header*)header_ptr;
      s_header->mark = FRAME_MARK;
      s_header->id_loc.fields.dev_id = dev_id & 0xF;
      s_header->id_loc.fields.flags = 0x0 & 0xF;
      s_header->id_loc.fields.data_code = d_code & 0xFF;
      s_header->num = fr_num;
      s_header->time = clock_get_time_s();
      return sizeof(type_SingleFrame_Header);
    case ARCH_HEADER_FRAME_TYPE:
      ah_header = (type_ArchHeadFrame_Header*)header_ptr;
      ah_header->mark = FRAME_MARK;
      ah_header->id_loc.fields.dev_id = dev_id & 0xF;
      ah_header->id_loc.fields.flags = 0x0 & 0xF;
      ah_header->id_loc.fields.data_code = d_code & 0xFF;
      ah_header->num = fr_num;
      ah_header->time = clock_get_time_s();
      ah_header->arch_len = num;
      return sizeof(type_ArchHeadFrame_Header);
    case ARCH_BODY_FRAME_TYPE:
      ab_header = (type_ArchBodyFrame_Header*)header_ptr;
      ab_header->mark = FRAME_MARK;
      ab_header->id_loc.fields.dev_id = dev_id & 0xF;
      ab_header->id_loc.fields.flags = (0x1 << 0) & 0xF;
      ab_header->id_loc.fields.data_code = d_code & 0xFF;
      ab_header->num = fr_num;
      ab_header->arch_num = num;
      return sizeof(type_ArchHeadFrame_Header);
    case DCR_FRAME_TYPE:
      dcr_header = (type_DCRFrame_Header*)header_ptr;
      dcr_header->mark = FRAME_MARK;
      dcr_header->id_loc.fields.dev_id = dev_id & 0xF;
      dcr_header->id_loc.fields.flags = 0x0 & 0xF;
      dcr_header->id_loc.fields.data_code = d_code & 0xFF;
      return sizeof(type_DCRFrame_Header);
  }
  return 0;
}

/**
  * @brief  подсчет и подстановка crc16 в гот овый кадр из 128 байт
  * @param  frame_ptr: указатель на кадр
  */
void frame_crc16_calc(uint8_t* frame_ptr)
{
  uint16_t crc16;
  crc16 = norby_crc16_calc(frame_ptr, 126);
  frame_ptr[126] = (crc16 >> 8) & 0xFF;
  frame_ptr[127] = (crc16 >> 0) & 0xFF;
}

/**
  * @brief  получение маяка в режиме констант
  * @param  frame_ptr: указатель на кадр
  */
void fill_beacon_const_mode(type_LM_Beacon_Frame* frame_ptr)
{
  frame_ptr->lm_status = 0x3D3E;
  frame_ptr->pl_status = 0x00;
  frame_ptr->lm_temp = 0x57;
  frame_ptr->pl_power_switches = 0x5C;
}

/**
  * @brief  получение тми в режиме констант
  * @param  frame_ptr: указатель на кадр
  */
void fill_tmi_const_mode(type_LM_TMI_Data_Frame* frame_ptr)
{
  uint8_t i=0;
  // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
  for(i=0; i<6; i++){
    frame_ptr->pl_status[i] = __REV16(((0x3D + (2*i+0)) << 8) + (0x3D + (2*i+1)));
  }
  for(i=0; i<7; i++){
    frame_ptr->pwr_inf[i].voltage = 0x49 + (2*i+0);
    frame_ptr->pwr_inf[i].current = 0x49 + (2*i+1);
  }
  for(i=0; i<5; i++){
    frame_ptr->temp[i] = 0x57 + i;
  }
  //
  frame_ptr->pl_power_switches = 0x5C;
  frame_ptr->iss_mem_status = 0x5D;
  frame_ptr->dcr_mem_status = 0x5E;
  frame_ptr->pl_rst_count = 0x5F;
  frame_ptr->com_reg_pwr_on_off = 0x60;
  frame_ptr->com_reg_inh = __REV16(0x6162);
  //
  frame_ptr->iss_rd_ptr = __REV16(0x6869);
  frame_ptr->iss_wr_ptr = __REV16(0x6A6B);
  frame_ptr->iss_mem_vol = __REV16(0x6C6D);
  frame_ptr->dcr_rd_ptr = __REV16(0x6E6F);
  frame_ptr->dcr_wr_ptr = __REV16(0x7071);
  frame_ptr->dct_mem_vol = __REV16(0x7273);
}
