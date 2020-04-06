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
extern type_LED_INDICATOR mcu_state_led, con_state_led;
//*** CallBack ***//
/**
  * @brief  регистрация колбэков для обработки команд CAN
  */
void ProcCallbackCmds_Init(void)
{
  interface_cb_registration(&lm.interface, ID_IVAR_CMD, ProcCallbackCmds);
  interface_cb_registration(&lm.interface, ID_IVAR_CMDREG, ProcCallbackCmdRegs);
  interface_cb_registration(&lm.interface, ID_IVAR_EXTMEM, ProcCallbackExtMems);
}

//*** набор callback-функция для бработки команд интерфейса общения с внешним миром (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state); ***//
/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  state: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackCmds(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
	uint8_t i;
  if(state == 0)
    return;  //первый вызов тут не нужен
  if (id.std.RTR == 1) return;  //обработка чтения нам не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      cmd_process_cb(&lm.interface, i);
  }
  printf("CAN%d DevId=%d VarId=%d Offset=%d RTR=%d Leng=%d State=%d\n\r", n, id.uf.DevId, id.uf.VarId, id.uf.Offset, id.uf.RTR, leng, state);
}

/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  state: тип CB - 0 до записи данных, 1 - после
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
  printf("CAN%d DevId=%d VarId=%d Offset=%d RTR=%d Leng=%d State=%d\n\r", n, id.uf.DevId, id.uf.VarId, id.uf.Offset, id.uf.RTR, leng, state);
}

/**
  * @brief  регистрация колбэк функции для чтения данных из памяти 3-х типов
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  state: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackExtMems(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
  if(state == 1) return;  //обработка второго вызова не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	if (id.uf.Offset%128 == 0){
      if(id.uf.Offset == 0){
        ext_mem_any_line_read(&lm.mem, lm.interface.ext_mem.External_Mem_Full_Frame);
      }
      else if(id.uf.Offset == 128){
        ext_mem_rd_frame_from_part(&lm.mem, lm.interface.ext_mem.External_Mem_ISS_Frame, PART_ISS);
      }
      else if(id.uf.Offset == 256){
        ext_mem_rd_frame_from_part(&lm.mem, lm.interface.ext_mem.External_Mem_DCR_Frame, PART_DCR);
      }
      printf("CAN%d DevId=%d VarId=%d Offset=%d RTR=%d Leng=%d State=%d\n\r", n, id.uf.DevId, id.uf.VarId, id.uf.Offset, id.uf.RTR, leng, state);
  }
}

//*** Cmds process function ***//
/**
  * @brief  функция для создания псевдопотока для выполнения продолжительной команды: test_led
  *         - раз в 2 секунды увеличивает скважность на 2 бит (100% - 255 бит)
  *         - полный цикл 200 секунд: от отсутствия мигания до постоянного горения
  * @param  cmd: команда, записанная в командный регистр
  * @param  period_ms: период вызова обработчика в псевдопотоке
  */
void cmd_process_test_led(uint8_t mode, uint32_t period_ms)
{
  //*** Прерывание работы ***//
  if (mode == MODE_CANCEL){
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, CMD_STATUS_CANCEL);
      lm.cmd_ctrl[CMD_TEST_LED].main_counter = 0;
      lm.cmd_ctrl[CMD_TEST_LED].time_ms = 0;
      lm.cmd_ctrl[CMD_TEST_LED].point_time_ms = 0;
      lm.cmd_ctrl[CMD_TEST_LED].ena = 0;
      return;
    }
  else{
    //*** Запуск работы ***//
    if ((mode == MODE_START) && (lm.cmd_ctrl[CMD_TEST_LED].ena == 0)){
      lm.cmd_ctrl[CMD_TEST_LED].main_counter = 0;
      lm.cmd_ctrl[CMD_TEST_LED].time_ms = 0;
      lm.cmd_ctrl[CMD_TEST_LED].point_time_ms = 0;
      lm.cmd_ctrl[CMD_TEST_LED].ena = 1;
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, CMD_STATUS_START);
      return;
    }
    //*** Собственно тело процесса ***//
    else if (lm.cmd_ctrl[CMD_TEST_LED].ena == 1){
      lm.cmd_ctrl[CMD_TEST_LED].time_ms += period_ms;
      lm.cmd_ctrl[CMD_TEST_LED].point_time_ms += period_ms;
      if (lm.cmd_ctrl[CMD_TEST_LED].point_time_ms >= 2000){
        lm.cmd_ctrl[CMD_TEST_LED].point_time_ms = 0;
        lm.cmd_ctrl[CMD_TEST_LED].main_counter += 2;
        led_alt_setup(&mcu_state_led, LED_BLINK, 500, (255 - lm.cmd_ctrl[CMD_TEST_LED].main_counter & 0xFF), 2000);
      }
      if (lm.cmd_ctrl[CMD_TEST_LED].main_counter >= 254){
        lm.cmd_ctrl[CMD_TEST_LED].ena = 2; // отправляем на окончание работы
      }
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, (lm.cmd_ctrl[CMD_TEST_LED].main_counter >> 2) & 0x7F);
    }
    //*** Окончание работы ***//
    else if (lm.cmd_ctrl[CMD_TEST_LED].ena == 2){
      lm.cmd_ctrl[CMD_TEST_LED].main_counter = 0;
      lm.cmd_ctrl[CMD_TEST_LED].time_ms = 0;
      lm.cmd_ctrl[CMD_TEST_LED].point_time_ms = 0;
      lm.cmd_ctrl[CMD_TEST_LED].ena = 0;
      cmd_set_status(&lm.interface, CMD_DBG_LED_TEST, CMD_STATUS_FINISH);
    }
  }
}
