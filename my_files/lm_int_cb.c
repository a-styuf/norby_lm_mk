/**
  ******************************************************************************
  * @file           : lm_int_cb.c
  * @version        : v1.0
  * @brief          : расширение для main, содержащие обработчики для callback-ов интерфейсов МС и обработчики команд
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "lm_int_cb.h"
#include <stdio.h>

extern type_LM_DEVICE lm;
//*** CallBack ***//
/**
  * @brief  регистрация колбэков для обработки команд CAN
  */
void ProcCallbackCmds_Init(void)
{
  interface_cb_registration(&lm.interface, ID_IVAR_CMD, ProcCallbackCmds);
  interface_cb_registration(&lm.interface, ID_IVAR_CMDREG, ProcCallbackCmdRegs);
}

//*** набор callback-функция для бработки команд интерфейса общения с внешним миром (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state); ***//
/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  tated: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackCmds(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
	uint8_t i;
  if(state == 0)
    return;  //первый вызов тут не нужен
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      cmd_process_cb(&lm.interface, i);
  }
  //printf("CAN%d DevId=%d VarId=%d Offset=%d RTR=%d Leng=%d State=%d\n\r", n, id.uf.DevId, id.uf.VarId, id.uf.Offset, id.uf.RTR, leng, state);
}

/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  tated: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackCmdRegs(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
	uint8_t i;
  if(state == 0) return;  //первый вызов тут не нужен
  if (id.std.RTR == 1) return;  //обработка чтения нам не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      cmdreg_process_cb(&lm.interface, i);
  }
  //printf("CAN%d DevId=%d VarId=%d Offset=%d RTR=%d Leng=%d State=%d\n\r", n, id.uf.DevId, id.uf.VarId, id.uf.Offset, id.uf.RTR, leng, state);
}

//*** Cmds process function ***//
void cmd_process_test_led(uint8_t mode)
{
  switch(mode){
    case MODE_START:
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, CMD_STATUS_START);
    break;
    case MODE_PROCESS:
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, CMD_STATUS_START); //:todo 
    break;
    case MODE_CANCEL:
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, CMD_STATUS_CANCEL);
    break;
    case MODE_IDLE:
      NULL;
    break;
  }
}