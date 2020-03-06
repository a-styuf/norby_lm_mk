#ifndef _LM_INTERFACES_H_
#define _LM_INTERFACES_H_

//*** include files ***//
#include "lm_interfaces_data.h"
#include "aa_can.h"
#include "main.h"

//*** defines ***//
// LM setup
//general setup
#define CAN_DEV_ID 0x06
#define CAN_BROADCAST_ID 0x0F
//VarId setup
#define ID_IVAR_PRG_MEM   00
#define ID_IVAR_RAM_MEM   01
#define ID_IVAR_CMD       02
#define ID_IVAR_CMD_STAT  03
#define ID_IVAR_CMD_REG   04
#define ID_IVAR_TMI       05
#define ID_IVAR_PARAM     06
#define ID_IVAR_EXTMEM    07
#define ID_IVAR_DBG       14
#define ID_IVAR_BRD       15
//
#define ID_IVAR_POOL_LEN  16
//
#define CMD_CNT           16
//*** structures ***//


/*------ Interface variables --------*/


#pragma pack(1)
/**
  * @brief  All command
  */
typedef struct {
  uint8_t CmdShort[CMD_CNT];  //+0
} type_IVar_Cmds;//CMD_CNT

/**
  * @brief  All command statuses
  */
typedef struct {
  uint8_t StatShort[CMD_CNT];  //+0
} type_IVar_Cmds_Statuses;//CMD_CNT

/**
  * @brief  All command registers
  */
typedef struct {
  uint8_t CmdShort1;  //+0
} type_IVar_Cmds_Registers;//1

/**
  * @brief  TMI data fieldes: contain data in 128-bytes form
  */
typedef struct {
  type_LM_Beacon_Frame beacon;  //+0
  type_LM_TMI_Data_Frame tmi; //+128
} type_IVar_TMI;//256

/**
  * @brief  Parameteres data
  */
typedef struct {
  type_LM_Parameters_Frame param;  //+0
} type_IVar_Param;//128

/**
  * @brief  External memory data in 128-bytes form
  */
typedef struct {
  uint8_t External_Mem_Frame[128];  //+0
} type_IVar_ExtMem; //128

/**
  * @brief  debug data
  */
typedef struct {
  uint8_t debug_data[128];  //+0
} type_IVar_DBG_Data; //128

#pragma pack(8) //default
/*----------------------------------------*/
typedef struct
{
  //
  type_IVar_Cmds cmd;
  type_IVar_Cmds_Statuses cmd_status;
  uint8_t cmd_to_check[CMD_CNT];
  uint8_t cmd_flg;
  type_IVar_Cmds_Registers cmd_reg;
  type_IVar_Param parameters;
  type_IVar_TMI tmi_data;
  type_IVar_ExtMem ext_mem;
  type_IVar_DBG_Data dbg_data;
  //
  typeRegistrationRec* reg_rec_ptr;
  CAN_TypeDef *can1_ptr, *can2_ptr;
} type_LM_INTERFACES;

//*** functions prototypes ***//
uint16_t interfaces_init(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev);
typeRegistrationRec _reg_rec_setup(void *VarPtr, uint32_t VarLeng, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state), uint8_t ivar_id, uint8_t access_flgs);
void interface_cb_registration(type_LM_INTERFACES* lm_in_ptr, uint8_t mode, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state));
uint16_t RegisterAllVars(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev);
#endif
