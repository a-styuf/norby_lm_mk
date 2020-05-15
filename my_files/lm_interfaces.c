/*
  ******************************************************************************
  * @file           : lm_interfaces.c
  * @version        : v1.0
  * @brief          : contain functions setup fo init and control all interfaces to outer world (CAN and USB-VCP)
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "lm_interfaces.h"

typeRegistrationRec RegistrationRec[ID_IVAR_POOL_LEN] = {0};
//extern typeRegistrationRec RegistrationRec[ID_IVAR_POOL_LEN];

/**
  * @brief  инициализация интерфейсов общения с МС (функции-обработчики регистрируются отдельно)
  * @retval статус ошибки: первый байт статус ошибки инициализации CAN1, второй - CAN2
  */
uint16_t interfaces_init(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev)
{
  uint8_t i;
  int8_t report = 0;
  uint16_t ret_val = 0;
  uint8_t var_array[ID_IVAR_POOL_LEN] = { ID_IVAR_PRG_MEM, 
                                          ID_IVAR_RAM_MEM, 
                                          ID_IVAR_CMD, 
                                          ID_IVAR_CMD_STAT, 
                                          ID_IVAR_CMDREG, 
                                          ID_IVAR_TMI,
                                          ID_IVAR_PARAM,
                                          ID_IVAR_EXTMEM,
                                          ID_IVAR_DCR_INTERFACE,
                                          ID_IVAR_ISS_INTERFACE,
                                          ID_IVAR_DBG,
                                          ID_IVAR_BRD, 
                                          ID_IVAR_FLASH_LOAD};
  lm_in_ptr->can1_ptr = CAN1;
  lm_in_ptr->can2_ptr = CAN2;
  lm_in_ptr->frame_num = 0x00;
  lm_in_ptr->reg_rec_ptr = RegistrationRec;
  //
  report = CAN_Init(lm_in_ptr->can1_ptr, CAN_SETUP_BTR);
  if (report) ret_val |= (1 << 14);
	report = CAN_Init(lm_in_ptr->can2_ptr, CAN_SETUP_BTR);
  if (report) ret_val |= (1 << 14);
  // variable registration teamplate
  for (i=0; i<ID_IVAR_POOL_LEN; i++)
  {
    switch(var_array[i]){
      case ID_IVAR_PRG_MEM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup((void*)0x8000000, 0, (void*)0, ID_IVAR_PRG_MEM, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_RAM_MEM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup((void*)0x2000000, 0, (void*)0, ID_IVAR_RAM_MEM, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_CMD:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->cmd, sizeof(type_IVar_Cmds), (void*)0, ID_IVAR_CMD, CAN_AFLG_READ_WRITE);
        break;
      case ID_IVAR_CMD_STAT:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->cmd_status, sizeof(type_IVar_Cmds_Statuses), (void*)0, ID_IVAR_CMD_STAT, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_CMDREG:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->cmdreg, sizeof(type_IVar_CmdRegisters), (void*)0, ID_IVAR_CMDREG, CAN_AFLG_READ_WRITE);
        break;
      case ID_IVAR_TMI:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->tmi_data, sizeof(type_IVar_TMI), (void*)0, ID_IVAR_TMI, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_PARAM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->parameters, sizeof(type_IVar_Param), (void*)0, ID_IVAR_PARAM, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_EXTMEM:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->ext_mem, sizeof(type_IVar_ExtMem), (void*)0, ID_IVAR_EXTMEM, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_DCR_INTERFACE:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->dcr_interface, sizeof(type_IVar_DCR_Interface), (void*)0, ID_IVAR_DCR_INTERFACE, CAN_AFLG_READ_WRITE);
        break;
      case ID_IVAR_ISS_INTERFACE:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->pl_iss_interface, sizeof(type_IVar_PL_ISS_Interface), (void*)0, ID_IVAR_ISS_INTERFACE, CAN_AFLG_READ_WRITE);
        break;
      case ID_IVAR_DBG:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->dbg_data, sizeof(type_IVar_DBG_Data), (void*)0, ID_IVAR_DBG, CAN_AFLG_READONLY);
        break;
      case ID_IVAR_BRD:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&lm_in_ptr->dbg_data, sizeof(type_IVar_DBG_Data), (void*)0, ID_IVAR_BRD, CAN_AFLG_READONLY); //todo
        break;
      case ID_IVAR_FLASH_LOAD:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup(&VarCAN_FlashFragment, 0x80008, (void*)0, ID_IVAR_FLASH_LOAD, CAN_AFLG_NOOFFSET);
        break;
      default:
        lm_in_ptr->reg_rec_ptr[i] = _reg_rec_setup((void*)0, 0, (void*)0, 0, CAN_AFLG_READONLY);
        break;
    }
  }
  //
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
  * @brief  регистрация переменных для can (заимстывованна из примера АА)
  * @param  id_dev: номер устройства на шине
  */
uint16_t RegisterAllVars(type_LM_INTERFACES* lm_in_ptr, uint8_t id_dev) 
{
  int n;
  int err = 0;
  for(n=0; n < ID_IVAR_POOL_LEN; n++) {
    err |= CAN_RegisterVar(n, id_dev) ? (1 << n) : err; 
  }
  return err;
}

/**
  * @brief  регистрация функции для переменной
  * @param  hal_can1_ptr: HAL-структура для управления CAN-ом №1
  */
void interface_cb_registration(type_LM_INTERFACES* lm_in_ptr, uint8_t mode, void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state))
{
  uint8_t number = 0;
  uint8_t var_array[ID_IVAR_POOL_LEN] = { ID_IVAR_PRG_MEM, 
                                          ID_IVAR_RAM_MEM, 
                                          ID_IVAR_CMD, 
                                          ID_IVAR_CMD_STAT, 
                                          ID_IVAR_CMDREG, 
                                          ID_IVAR_TMI,
                                          ID_IVAR_PARAM,
                                          ID_IVAR_EXTMEM,
                                          ID_IVAR_DCR_INTERFACE,
                                          ID_IVAR_ISS_INTERFACE,
                                          ID_IVAR_DBG,
                                          ID_IVAR_BRD, 
                                          ID_IVAR_FLASH_LOAD};
  //
  for(number=0; number<sizeof(var_array); number++){
    if (mode == var_array[number]){
      break;
    }
  }
  lm_in_ptr->reg_rec_ptr[number].CallBackProc = CallBackProc;
}

///*** Commands ***///

/**
  * @brief  обработка команды при приеме регистра команд в колбэке
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @param  cmd_num номер команды
  */
void cmd_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num)
{
  switch(lm_in_ptr->cmd.array[cmd_num]){
    case CMD_STATUS_CLEAR:
      cmd_set_status(lm_in_ptr, cmd_num, CMD_STATUS_CLEAR);
    break;
    case CMD_STATUS_START:
      lm_in_ptr->cmd_to_check[cmd_num] = lm_in_ptr->cmd.array[cmd_num];
      lm_in_ptr->cmd_flg += 1;
      cmd_set_status(lm_in_ptr, cmd_num, CMD_STATUS_START);
    break;
    case CMD_STATUS_CANCEL:
      lm_in_ptr->cmd_to_check[cmd_num] = lm_in_ptr->cmd.array[cmd_num];
      lm_in_ptr->cmd_flg += 1;
      cmd_set_status(lm_in_ptr, cmd_num, CMD_STATUS_CANCEL);
    break;
  }
}

/**
  * @brief  проверка на обработку команды для main
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @retval -1: нет команд; другое: номер команды 
  */
int16_t cmd_check_to_process(type_LM_INTERFACES* lm_in_ptr)
{
  if (lm_in_ptr->cmd_flg == 0){
    return -1;
  }
  else {
    for (int i=0; i<CMD_POOL_LEN; i++){
      if (lm_in_ptr->cmd_to_check[i]){
        lm_in_ptr->cmd_to_check[i] = 0;
        lm_in_ptr->cmd_flg -= 1;
        return i;
      }
    }
    lm_in_ptr->cmd_flg -= 1;
  }
	return -1;
}

/**
  * @brief  установка статуса команды
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @param  cmd_num номер команды
  * @param  status номер команды
  */
void cmd_set_status(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num, uint8_t cmd_status)
{
  if (cmd_num < CMD_POOL_LEN){
    lm_in_ptr->cmd_status.array[cmd_num] = cmd_status;
  }
}

///*** Command registers ***///

/**
  * @brief  обработка записи в командные регистры при приеме регистра команд в колбэке
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @param  cmd_num номер командног регистра
  */
void cmdreg_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t cmd_num)
{
      lm_in_ptr->cmdreg_to_check[cmd_num] = 0x01;
      lm_in_ptr->cmdreg_flg += 1;
}

/**
  * @brief  проверка на обработку командного регистра для main
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @retval -1: нет команд; другое: номер командного регистра 
  */
int16_t cmdreg_check_to_process(type_LM_INTERFACES* lm_in_ptr)
{
  if (lm_in_ptr->cmdreg_flg == 0){
    return -1;
  }
  else {
    for (int i=0; i<CMDREG_POOL_LEN; i++){
      if (lm_in_ptr->cmdreg_to_check[i]){
        lm_in_ptr->cmdreg_to_check[i] = 0;
        lm_in_ptr->cmdreg_flg -= 1;
        return i;
      }
    }
    lm_in_ptr->cmdreg_flg -= 1;
  }
	return -1;
}

///*** DCR Interface ***///
/**
  * @brief  обработка записи в регистр интерфейса к ДеКоР
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @param  offset смещение записанного регистра
  * @note   запуск немедленной отправки осуществляется записью длины в поле длины (смещение 127), остальные  возможные команды 
  *         планируется выполнять через регистр статусов
  */
void dcr_inerface_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t offset)
{
  switch(offset){
    case DCR_INTERFACE_INSTASEND_LENG_OFFSET:
      //для немедленной отправки сообщения проверяем только поле длины (offset = 127)
      lm_in_ptr->dcr_int_offset_to_check[offset] = 0x01;
      lm_in_ptr->dcr_int_flg += 1;
      break;
  }
}


/**
  * @brief  проверка на обработку командного регистра для интерфейса к ДеКоР
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @retval -1: нет команд; другое: номер командного регистра 
  */
int16_t dcr_inerface_check_to_process(type_LM_INTERFACES* lm_in_ptr)
{
  if (lm_in_ptr->dcr_int_flg == 0){
    return -1;
  }
  else {
    for (int i=0; i<DCR_INTERFACE_CMD_POOL_LEN; i++){
      if (lm_in_ptr->dcr_int_offset_to_check[i]){
        lm_in_ptr->dcr_int_offset_to_check[i] = 0;
        lm_in_ptr->dcr_int_flg -= 1;
        return i;
      }
    }
    lm_in_ptr->dcr_int_flg -= 1;
  }
	return -1;
}

///*** PL_ISS Interface ***///
/**
  * @brief  обработка записи в регистр интерфейса к ПНН_ИСС
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @param  offset смещение записанного регистра
  * @note   запуск немедленной отправки осуществляется записью длины в поле длины (смещение 127), остальные  возможные команды 
  *         планируется выполнять через регистр статусов
  */
void pl_iss_inerface_process_cb(type_LM_INTERFACES* lm_in_ptr, uint16_t offset)
{
  switch(offset){
    case PL11A_INTERFACE_INSTASEND_LENG_OFFSET:
    case PL11B_INTERFACE_INSTASEND_LENG_OFFSET:
    case PL12_INTERFACE_INSTASEND_LENG_OFFSET:
    case PL20_INTERFACE_INSTASEND_LENG_OFFSET:
      //для немедленной отправки сообщения проверяем только поле длины (offset = 127)
      lm_in_ptr->pl_iss_int_offset_to_check[offset] = 0x01;
      lm_in_ptr->pl_iss_int_flg += 1;
      break;
  }
}


/**
  * @brief  проверка на обработку командного регистра для интерфейса к ДеКоР
  * @param  lm_in_ptr указатель на сруктуру с интерфейсом
  * @retval -1: нет команд; другое: номер командного регистра 
  */
int16_t pl_iss_inerface_check_to_process(type_LM_INTERFACES* lm_in_ptr)
{
  if (lm_in_ptr->pl_iss_int_flg == 0){
    return -1;
  }
  else {
    for (int i=0; i<PL_ISS_INTERFACE_CMD_POOL_LEN; i++){
      if (lm_in_ptr->pl_iss_int_offset_to_check[i]){
        lm_in_ptr->pl_iss_int_offset_to_check[i] = 0;
        lm_in_ptr->pl_iss_int_flg -= 1;
        return i;
      }
    }
    lm_in_ptr->pl_iss_int_flg -= 1;
  }
	return -1;
}

