/**
  ******************************************************************************
  * @file           : lm_interfaces.c
  * @version        : v1.0
  * @brief          : contain functions setup fo init and control all interfaces to outer world (CAN and USB-VCP)
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "lm_interfaces.h"

extern typeRegistrationRec RegistrationRec[ID_IVAR_POOL_LEN];

/**
  * @brief  инициализация интерфейсов общения с МС (функции-обработчики регистрируются отдельно)
  * @retval статус ошибки: первый байт статус ошибки инициализации CAN1, второй - CAN2
  */
uint16_t interfaces_init(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev)
{
  uint8_t i;
  int8_t report = 0;
  uint16_t ret_val = 0;
  lm_in_ptr->can1_ptr = CAN1;
  lm_in_ptr->can2_ptr = CAN2;
  lm_in_ptr->reg_rec_ptr = RegistrationRec;
	report = CAN_Init(lm_in_ptr->can1_ptr);
  if (report) return report;
	report = CAN_Init(lm_in_ptr->can2_ptr);
  if (report) return report;
  // variable registration teamplate
  for (i=0; i<ID_IVAR_POOL_LEN; i++)
  {
    switch(i){
      case ID_IVAR_PRG_MEM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup((void*)0x8000000, 0, (void*)0, 0, 1);
        break;
      case ID_IVAR_RAM_MEM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup((void*)0x2000000, 0, (void*)0, 0, 1);
        break;
      case ID_IVAR_CMD:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->cmd, sizeof(type_IVar_Cmds), (void*)0, ID_IVAR_CMD, 0);
        break;
      case ID_IVAR_CMD_STAT:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->cmd_status, sizeof(type_IVar_Cmds_Statuses), (void*)0, ID_IVAR_CMD_STAT, 1);
        break;
      case ID_IVAR_CMD_REG:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->cmd_reg, sizeof(type_IVar_Cmds_Registers), (void*)0, ID_IVAR_CMD_REG, 0);
        break;
      case ID_IVAR_TMI:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->tmi_data, sizeof(type_IVar_TMI), (void*)0, ID_IVAR_TMI, 1);
        break;
      case ID_IVAR_PARAM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->parameters, sizeof(type_IVar_Param), (void*)0, ID_IVAR_PARAM, 1);
        break;
      case ID_IVAR_EXTMEM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->ext_mem, sizeof(type_IVar_ExtMem), (void*)0, ID_IVAR_EXTMEM, 1);
        break;
      case ID_IVAR_DBG:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->dbg_data, sizeof(ID_IVAR_DBG), (void*)0, ID_IVAR_DBG, 1);
        break;
      case ID_IVAR_BRD:
        break;
      default:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup((void*)0, 0, (void*)0, 0, 1);
        break;
    }
  }
  ret_val = RegisterAllVars(lm_in_ptr, id_dev);
  if (ret_val) return ret_val;
  return 0;
}


/**
  * @brief  регистрация переменной (заполнени структуры typeRegistrationRec)
  * @param  VarPtr указатель на IVar
  * @param  VarLeng размер IVar
  * @param  CallBackProc функция обработчик записи/чтения с параметрами (CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state)
  * @param  ivar_id размер IVar
  * @param  access_flgs тип доступа к переменной: [0]=1 - read only, [1]=1 - regardless offset, [2]=1 - without filter
  * @retval typeRegistrationRec переменная с полями из параметров функций
  */
typeRegistrationRec _reg_rec_setup(void *VarPtr, uint32_t VarLeng, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state), uint8_t ivar_id, uint8_t access_flgs)
{
  typeRegistrationRec ret_registration_rec = {VarPtr, VarLeng, CallBackProc, ivar_id, access_flgs};
  return ret_registration_rec;
}

/**
  * @brief  регистрация функции для переменной
  * @param  hal_can1_ptr: HAL-структура для управления CAN-ом №1
  */
void interface_cb_registration(type_LM_INTERFACES* lm_in_ptr, uint8_t mode, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state))
{
  lm_in_ptr->reg_rec_ptr[mode].CallBackProc = CallBackProc;
}

/**
  * @brief  регистрация переменных для can
  * @param  id_dev: номер устройства на шине
  */
uint16_t RegisterAllVars(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev) 
{
  int n;
  int err = 0;
  for(n=0; n < 16; n++) {
    err |= (1 << n) ? CAN_RegisterVar(n, id_dev) : err; 
  }
  return err;
}