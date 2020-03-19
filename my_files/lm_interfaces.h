#ifndef _LM_INTERFACES_H_
#define _LM_INTERFACES_H_

//*** include files ***//
#include "lm_interfaces_data.h"
#include "aa_can.h"
#include "main.h"

//*** defines ***//
// LM setup
//***General setup
#define CAN_BROADCAST_ID 0x0F
//***VarId setup
#define ID_IVAR_PRG_MEM   00
#define ID_IVAR_RAM_MEM   01
#define ID_IVAR_CMD       02
#define ID_IVAR_CMD_STAT  03
#define ID_IVAR_CMDREG    04
#define ID_IVAR_TMI       05
#define ID_IVAR_PARAM     06
#define ID_IVAR_EXTMEM    07
#define ID_IVAR_DBG       12
#define ID_IVAR_BRD       13
//
#define ID_IVAR_POOL_LEN  14
//***Cmds defines
#define CMD_INIT_LM       0x00
#define CMD_INIT_ISS_MEM  0x01
#define CMD_INIT_DCR_MEM  0x02
#define CMD_DBG_LED_TEST  0x10
//
#define CMD_POOL_LEN      32
//
#define CMD_NO_ONE        -1
// default cmd statuses
#define CMD_STATUS_CLEAR  0x00
#define CMD_STATUS_START  0x01
#define CMD_STATUS_FINISH 0x7F
#define CMD_STATUS_CANCEL 0xFF
// default cmd
#define CMD_CLEAR  0x00
#define CMD_START  0x01
#define CMD_CANCEL 0xFF
//***CmdsReg setup
#define CMDREG_LM_MODE    0x00
#define CMDREG_PL_PWR_SW  0x01
#define CMDREG_PL_INH_0   0x02
#define CMDREG_PL_INH_1   0x03
#define CMDREG_DBG_LED    0x10
//
#define CMDREG_POOL_LEN   32
//*** structures ***//


/*------ Interface variables --------*/


#pragma pack(2)
/**
  * @brief  All command
  */
typedef struct {
  uint8_t array[CMD_POOL_LEN];  //+0
} type_IVar_Cmds;//CMD_CNT

/**
  * @brief  All command statuses
  */
typedef struct {
  uint8_t array[CMD_POOL_LEN];  //+0
} type_IVar_Cmds_Statuses;//CMD_CNT

/**
  * @brief  All command registers
  */
typedef struct {
  uint8_t array[CMDREG_POOL_LEN];  //+0
} type_IVar_CmdRegisters;//CMDREG_POOL_LEN

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
  uint8_t cmd_to_check[CMD_POOL_LEN];
  uint16_t cmd_flg;
  type_IVar_CmdRegisters cmdreg;
  uint8_t cmdreg_to_check[CMDREG_POOL_LEN];
  uint16_t cmdreg_flg;
  type_IVar_Param parameters;
  type_IVar_TMI tmi_data;
  type_IVar_ExtMem ext_mem;
  type_IVar_DBG_Data dbg_data;
  //
  typeRegistrationRec* reg_rec_ptr;
  CAN_TypeDef *can1_ptr, *can2_ptr;
  uint16_t frame_num;
} type_LM_INTERFACES;

//*** functions prototypes ***//
uint16_t interfaces_init(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev);
typeRegistrationRec _reg_rec_setup(void *VarPtr, uint32_t VarLeng, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state), uint8_t ivar_id, uint8_t access_flgs);
uint16_t RegisterAllVars(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev);
void interface_cb_registration(type_LM_INTERFACES* lm_in_ptr, uint8_t mode, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state));
//работа с командами
void cmd_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num);
int16_t cmd_check_to_process(type_LM_INTERFACES* lm_in_ptr);
void cmd_set_status(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num, uint8_t cmd_status);
void cmdreg_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num);
int16_t cmdreg_check_to_process(type_LM_INTERFACES* lm_in_ptr);

#endif
