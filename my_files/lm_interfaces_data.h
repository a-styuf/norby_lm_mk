#ifndef _LM_INTERFACES_DATA_H_
#define _LM_INTERFACES_DATA_H_

//*** Includes ***//
#include "crc16.h"

//*** Defines ***//
#define FRAME_MARK 0x0FF1

#define SINGLE_FRAME_TYPE       0x00
#define ARCH_HEADER_FRAME_TYPE  0x01
#define ARCH_BODY_FRAME_TYPE    0x02

#define DATA_TYPE_BEACON        0x80
#define DATA_TYPE_TMI           0x81

//*** General parts of frames ***//
#pragma pack(2)
/**
  * @brief  Union for easy acces to id_loc fields (2 bytes)
  */
typedef union{
  uint16_t full;
  struct {
    uint16_t data_code : 8;
    uint16_t flags : 4;
    uint16_t dev_id : 4;
  }fields;
} type_ID_Loc; //2

/**
  * @brief  single frame header type (10 bytes)
  */
typedef struct {
  uint16_t mark; //+0
  type_ID_Loc id_loc; //+2
  uint16_t num; //+4
  uint32_t time; //+6
} type_SingleFrame_Header; //10

/**
  * @brief  head of archive header type (12 bytes)
  */
typedef struct {
  uint16_t mark; //+0
  type_ID_Loc id_loc; //+2
  uint16_t num; //+4
  uint32_t time; //+6
  uint16_t arch_len; //+10
} type_ArchHeadFrame_Header; //12

/**
  * @brief  body of archive header type (8 bytes)
  */
typedef struct {
  uint16_t mark; //+0
  type_ID_Loc id_loc; //+2
  uint16_t num; //+4
  uint16_t arch_num; //+6
} type_ArchBodyFrame_Header; //8

/**
  * @brief  struct to store PL-power information
  */
typedef struct{
  uint8_t voltage;
  uint8_t current;
} type_POWER; //2

//***  Frame teamplates  ***//
/**
  * @brief  Beacon data in 128-bytes single_data form
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  //
  uint16_t lm_status; //+10
  uint16_t pl_status; //+12
  uint8_t lm_temp; //+11
  uint8_t pl_power_switches; //+14
  //
  uint8_t filler[110]; //+16
  //
  uint16_t crc16; //+126
} type_LM_Beacon_Frame; //128

/**
  * @brief  TMI data in 128-bytes single_data form
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР1, 6-ПН_ДКР2
  uint16_t pl_status[6];  //+10
  type_POWER pwr_inf[7];  //+22
  uint8_t temp[5];        //+36
  //
  uint8_t pl_power_switches; //+41
  uint8_t iss_mem_status; //+42
  uint8_t dcr_mem_status; //+43
  uint8_t pl_rst_count; //+44
  uint8_t gap; //+45
  //
  uint8_t filler[80]; //+46
  //
  uint16_t crc16; //+126
} type_LM_TMI_Data_Frame; //128

/**
  * @brief  Parameters data
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  uint8_t filler[116]; //+10
  uint16_t crc16; //+126
} type_LM_Parameters_Frame; //128

#pragma pack(8)
//*** Function prototypes ***//
uint8_t frame_create_header(uint8_t* header_ptr, uint8_t dev_id, uint8_t type, uint8_t d_code, uint16_t fr_num, uint16_t num, uint32_t time_s);
void frame_crc16_calc(uint8_t* header_ptr);

#endif
