#ifndef _LM_INTERFACES_DATA_H_
#define _LM_INTERFACES_DATA_H_

//*** Includes ***//
#include "crc16.h"

//*** Defines ***//
#define FRAME_MARK 0x0FF1

#define SINGLE_FRAME_TYPE       0x00
#define ARCH_HEADER_FRAME_TYPE  0x01
#define ARCH_BODY_FRAME_TYPE    0x02

//*** General parts of frames ***//
#pragma pack(1)
/**
  * @brief  Union for easy acces to id_loc fields (2 bytes)
  */
typedef union{
  uint16_t full;
  struct {
    uint16_t dev_id : 4;
    uint16_t flags : 4;
    uint16_t data_code : 8;
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

//***  Frame teamplates  ***//
/**
  * @brief  Beacon data in 128-bytes single_data form
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  uint8_t filler[116]; //+10
  uint16_t crc16; //+126
} type_LM_Beacon_Frame; //128

/**
  * @brief  TMI data in 128-bytes single_data form
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  uint8_t filler[116]; //+10
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

#pragma pack(2)
//*** Function prototypes ***//
uint8_t frame_create_header(uint8_t* header_ptr, uint8_t dev_id, uint8_t type, uint8_t d_code, uint16_t* fr_num, uint16_t num, uint32_t time_s);
void frame_crc16_calc(uint8_t* header_ptr);

#endif
