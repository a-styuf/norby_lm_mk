#ifndef _LM_INTERFACES_DATA_H_
#define _LM_INTERFACES_DATA_H_

//*** Includes ***//
#include "crc16.h"
#include "clock.h"

//*** Defines ***//
// определитель кадров (метка для визуального или автоматического поиска кадров)
#define FRAME_MARK 0x0FF1
// типы кадров 
#define SINGLE_FRAME_TYPE       0x00
#define ARCH_HEADER_FRAME_TYPE  0x01
#define ARCH_BODY_FRAME_TYPE    0x02
#define DCR_FRAME_TYPE          0x03
// типы данных в кадрах
#define DATA_TYPE_BEACON              0x80
#define DATA_TYPE_TMI                 0x81
#define DATA_TYPE_GEN_TMI             0x82
#define DATA_TYPE_DCR_LAST_RX_FRAME   0x83
#define DATA_TYPE_DCR_LAST_RX_STATUS  0x84
#define DATA_TYPE_PL11A_INT_DATA      0x85
#define DATA_TYPE_PL11B_INT_DATA      0x86
#define DATA_TYPE_PL12_INT_DATA       0x87
#define DATA_TYPE_PL20_INT_DATA       0x88
#define DATA_TYPE_CYCLOGRAM_RESULT    0x89
#define DATA_TYPE_LOAD_PARAM          0x8A

#define DATA_TYPE_LM_CONFIG           0x8F



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
  * @brief  body of archive data type (8 bytes)
  */
typedef struct {
  uint16_t mark; //+0
  type_ID_Loc id_loc; //+2
  uint16_t num; //+4
  uint16_t arch_num; //+6
} type_ArchBodyFrame_Header; //8

/**
  * @brief  body of long DCR-frame (4 bytes)
  */
typedef struct {
  uint16_t mark; //+0
  type_ID_Loc id_loc; //+2
} type_DCRFrame_Header; //8

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
  uint8_t lm_temp; //+14
  uint8_t pl_power_switches; //+15
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
  uint16_t pl_status[6];    //+10
  type_POWER pwr_inf[7];    //+22
  uint8_t temp[5];          //+36
  //
  uint8_t pl_power_switches;  //+41
  uint8_t iss_mem_status;   //+42
  uint8_t dcr_mem_status;   //+43
  uint8_t pl_rst_count;   //+44
  uint8_t com_reg_pwr_on_off;   //+45
  uint16_t com_reg_inh;  //+46
  uint16_t iss_rd_ptr;    //+48
  uint16_t iss_wr_ptr;    //+50
  uint16_t iss_mem_vol;    //+52
  uint16_t dcr_rd_ptr;    //+54
  uint16_t dcr_wr_ptr;    //+56
  uint16_t dct_mem_vol;    //+58
  uint16_t rsrv[5]; //+60
  //
  uint8_t filler[56]; //+70
  //
  uint16_t crc16; //+126
} type_LM_TMI_Data_Frame; //128

/**
  * @brief  Full LM TMI data in 128-bytes single_data form
  * @note   aLL system reports are contained in 18-bytes fields
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР
  uint8_t lm_report[18]; //+10
  uint8_t pl11a_report[18]; //+28
  uint8_t pl11b_report[18]; //+46
  uint8_t pl12_report[18]; //+64
  uint8_t pl20_report[18]; //+82
  uint8_t pldcr_report[18]; //+100
  uint16_t iss_rd_ptr; //+118
  uint16_t iss_mem_vol; //+120
  uint16_t dcr_rd_ptr; //+122
  uint16_t dct_mem_vol; //+124
  //
  uint16_t crc16; //+126
} type_LM_GEN_TMI_Frame; //128

/**
  * @brief  Load parameters of LM
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  uint16_t version[3];          //+10
  int16_t pwr_init_report;      //+16
  int16_t tmp_init_report;      //+18
  int16_t cycl_init_report;     //+20
  int16_t int_init_report;      //+22
  int16_t ext_mem_init_report;  //+24
  //
  uint8_t filler[100];          //+26
  //
  uint16_t crc16;               //+126
} type_LM_LOAD_PARAM_Frame;      //128

/**
  * @brief  DCR-data frame
  * @note   long_frame (128-bytes) for saving to ext_mem and share to can-mem
  */
typedef struct {
  type_DCRFrame_Header header; //+0
  
  uint8_t long_dcr_frame[124]; //+4
  //
} type_DCR_LONG_Frame; //128

/**
  * @brief  DCR-status
  * @note   status_frame (up to 116-bytes) for share to can-mem
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  //
  uint8_t status_dcr_data[116]; //+10
  //
  uint16_t crc16; //+126
} type_DCR_STATUS_Frame; //128

/**
  * @brief  PL1.1_A, PL1.1_B, PL1.2, PL2.0 interface answer
  * @note   answer from PL1-ISS from UART must be less than 116-byte
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  //
  uint8_t data[116]; //+10
  //
  uint16_t crc16; //+126
} type_PL_ISS_INT_data; //128


/**
  * @brief  pn1.1 cyclogram result header
  * @note   format 
  */
typedef struct {
  type_ArchHeadFrame_Header header;   //+0
  uint16_t result_num;                //+12
  uint8_t cyclograma_mode;            //+14
  uint8_t cyclograma_status;          //+15
  uint8_t reserved[14];               //+16
  uint8_t tmi_slice[8][12];           //+30 //leng = 8*12=96
  //
  uint16_t crc16;                     //+126
} type_PL_CYCLOGRAMA_RESULT_HEADER; //128

/**
  * @brief  pn1.1 cyclogram result body
  * @note   format 
  */
typedef struct {
  type_ArchBodyFrame_Header header;   //+0
  uint8_t data[118];                  //+8
  uint16_t crc16;                     //+126
} type_PL_CYCLOGRAMA_RESULT_BODY; //128

/**
  * @brief  Parameters data
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  uint8_t filler[116]; //+10
  uint16_t crc16; //+126
} type_LM_Parameters_Frame; //128


/**
  * @brief  Config parameters to save in special memory part
  * @note   aLL system cfg are contained in 18-bytes fields
  */
typedef struct {
  type_SingleFrame_Header header; //+0
  // 0-МС, 1-ПН1.1A, 2-ПН1.1В, 3-ПН1.2, 4-ПН2.0, 5-ПН_ДКР
  uint8_t lm_cfg[26]; //+10
  uint8_t pl11a_cfg[18]; //+36
  uint8_t pl11b_cfg[18]; //+54
  uint8_t pl12_cfg[18]; //+72
  uint8_t pl20_cfg[18]; //+90
  uint8_t pldcr_cfg[18]; //+108
  //
  uint16_t crc16; //+126
} type_LM_CFG_Frame; //128

#pragma pack(8)
//*** Function prototypes ***//
uint8_t frame_create_header(uint8_t* header_ptr, uint8_t dev_id, uint8_t type, uint8_t d_code, uint16_t fr_num, uint16_t num);
void frame_crc16_calc(uint8_t* header_ptr);

void fill_beacon_const_mode(type_LM_Beacon_Frame* frame_ptr);
void fill_tmi_const_mode(type_LM_TMI_Data_Frame* frame_ptr);
#endif
