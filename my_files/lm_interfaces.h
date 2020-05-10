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
#define ID_IVAR_PRG_MEM       0
#define ID_IVAR_RAM_MEM       1
#define ID_IVAR_CMD           2
#define ID_IVAR_CMD_STAT      3
#define ID_IVAR_CMDREG        4
#define ID_IVAR_TMI           5
#define ID_IVAR_PARAM         6
#define ID_IVAR_EXTMEM        7
#define ID_IVAR_DCR_INTERFACE 8
#define ID_IVAR_ISS_INTERFACE 9
#define ID_IVAR_DBG           12
#define ID_IVAR_BRD           13
//
#define ID_IVAR_POOL_LEN      14
//***Cmds defines
#define CMD_INIT_LM                     0x00
#define CMD_INIT_ISS_MEM                0x01
#define CMD_INIT_DCR_MEM                0x02
#define CMD_DCR_WRITE_FLIGHT_TASK       0x03
#define CMD_DBG_LED_TEST                0x10
//
#define CMD_POOL_LEN          32
//
#define CMD_NO_ONE            -1
// default cmd statuses
#define CMD_STATUS_CLEAR           0x00
#define CMD_STATUS_START           0x01
#define CMD_STATUS_FINISH          0x7F
#define CMD_STATUS_CANCEL          0xFF
// default cmd
#define CMD_CLEAR                 0x00
#define CMD_START                 0x01
#define CMD_CANCEL                0xFF
//***CmdsReg setup
#define CMDREG_LM_MODE            0x00
#define CMDREG_PL_PWR_SW          0x01
#define CMDREG_PL_INH_0           0x02
#define CMDREG_PL_INH_1           0x03
//
#define CMDREG_ALL_MEM_RD_PTR_0   0x04
#define CMDREG_ALL_MEM_RD_PTR_1   0x05
#define CMDREG_ALL_MEM_RD_PTR_2   0x06
#define CMDREG_ALL_MEM_RD_PTR_3   0x07
//
#define CMDREG_DCR_MODE_SET       0x08
//
#define CMDREG_PL11A_OUT_SET      0x09
#define CMDREG_PL11B_OUT_SET      0x0A  //10
#define CMDREG_PL12_OUT_SET       0x0B  //11
#define CMDREG_PL20_OUT_SET       0x0C  //12

//
#define CMDREG_DBG_LED            0x10
#define CMDREG_DBG_CYCLOGRAMS_0   0x11
#define CMDREG_DBG_CYCLOGRAMS_1   0x12
//
#define CMDREG_POOL_LEN   32
//***DCR_Interface setup
#define DCR_INTERFACE_INSTASEND_LENG_OFFSET   127

#define DCR_INTERFACE_CMD_POOL_LEN   128

//***PL_ISS_Interface setup
#define PL11A_INTERFACE_INSTASEND_LENG_OFFSET   (0*128 + 127)
#define PL11B_INTERFACE_INSTASEND_LENG_OFFSET   (1*128 + 127)
#define PL12_INTERFACE_INSTASEND_LENG_OFFSET    (2*128 + 127)
#define PL20_INTERFACE_INSTASEND_LENG_OFFSET    (3*128 + 127)

#define PL_ISS_INTERFACE_CMD_POOL_LEN           (4*128)
//

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
  type_LM_Beacon_Frame beacon;          //+0
  type_LM_TMI_Data_Frame tmi;           //+128
  type_LM_GEN_TMI_Frame gen_tmi;        //+256
  type_DCR_STATUS_Frame dcr_status;     //+384
  type_DCR_LONG_Frame dcr_frame;        //+512
  type_PL_ISS_INT_data pl11a_frame;   //+640
  type_PL_ISS_INT_data pl11b_frame;   //+768
  type_PL_ISS_INT_data pl12_frame;    //+896
  type_PL_ISS_INT_data pl20_frame;    //+1024
} type_IVar_TMI;                        //1152

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
  uint8_t External_Mem_Full_Frame[128];  //+0
  uint8_t External_Mem_ISS_Frame[128];  //+128
  uint8_t External_Mem_DCR_Frame[128];  //+256
} type_IVar_ExtMem; //384



/**
  * @brief  IVar for DCR-interface
  */
typedef struct {
  uint8_t InstaMessage[128];  //+0
  uint8_t FlightTask[128][16];  //+128
} type_IVar_DCR_Interface; //2176

/**
  * @brief  IVar for PL_ISS-interface
  */
typedef struct {
  uint8_t InstaMessage[4][128];   //+0
} type_IVar_PL_ISS_Interface;        //128*4

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
  //
  type_IVar_CmdRegisters cmdreg;
  uint8_t cmdreg_to_check[CMDREG_POOL_LEN];
  uint16_t cmdreg_flg;
  //
  type_IVar_Param parameters;
  //
  type_IVar_TMI tmi_data;
  //
  type_IVar_ExtMem ext_mem;
  //
  type_IVar_DCR_Interface dcr_interface;
  uint8_t dcr_int_offset_to_check[DCR_INTERFACE_CMD_POOL_LEN];
  uint16_t dcr_int_flg;
  //
  type_IVar_PL_ISS_Interface pl_iss_interface;
  uint8_t pl_iss_int_offset_to_check[PL_ISS_INTERFACE_CMD_POOL_LEN];
  uint16_t pl_iss_int_flg;
  //
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
//работа с командыми регистрами
void cmdreg_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num);
int16_t cmdreg_check_to_process(type_LM_INTERFACES* lm_in_ptr);
//работа с командами интерфейса к ДеКоР
void dcr_inerface_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t offset);
int16_t dcr_inerface_check_to_process(type_LM_INTERFACES* lm_in_ptr);
//работа с командами интерфейса к ПН_ИСС
void pl_iss_inerface_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t offset);
int16_t pl_iss_inerface_check_to_process(type_LM_INTERFACES* lm_in_ptr);

#endif
