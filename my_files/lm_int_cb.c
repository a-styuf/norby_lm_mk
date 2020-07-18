/**
  ******************************************************************************
  * @file           : lm_int_cb.c
  * @version        : v1.0
  * @brief          : расширение для main, содержащие обработчики для callback-ов интерфейсов МС и обработчики команд
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "lm_int_cb.h"
#include "led.h"

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
  interface_cb_registration(&lm.interface, ID_IVAR_DCR_INTERFACE, ProcCallbackDCRInterface);
  interface_cb_registration(&lm.interface, ID_IVAR_ISS_INTERFACE, ProcCallbackISSInterface);
  interface_cb_registration(&lm.interface, ID_IVAR_FLASH_LOAD, ProcCallbackCAN_Flash);
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
	uint32_t i;
  if(state == 0)
    return;  //первый вызов тут не нужен
  if (id.std.RTR == 1) return;  //обработка чтения нам не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      cmd_process_cb(&lm.interface, i);
  }
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
	uint32_t i;
  if(state == 0) return;  //первый вызов тут не нужен
  if (id.std.RTR == 1) return;  //обработка чтения нам не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      cmdreg_process_cb(&lm.interface, i);
  }
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
  uint8_t offset;
  if(state == 1) {
    return;  //обработка второго вызова не нужна
  }
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
  offset = id.uf.Offset % 128;
	if (id.uf.Offset % 8 == 0){
      if((id.uf.Offset >= 128*0) && (id.uf.Offset < 128*1)){
        ext_mem_read_from_part_8b(&lm.mem, offset, lm.interface.ext_mem.External_Mem_Full_Frame + offset, PART_ALL_MEM);
      }
      else if((id.uf.Offset >= 128*1) && (id.uf.Offset < 128*2)){
        ext_mem_read_from_part_8b(&lm.mem, offset, lm.interface.ext_mem.External_Mem_ISS_Frame + offset, PART_ISS);
      }
      else if((id.uf.Offset >= 128*2) && (id.uf.Offset < 128*3)){
        ext_mem_read_from_part_8b(&lm.mem, offset, lm.interface.ext_mem.External_Mem_DCR_Frame + offset, PART_DCR);
      }
      else if((id.uf.Offset >= 128*3) && (id.uf.Offset < 128*4)){
        ext_mem_read_from_part_8b(&lm.mem, offset, lm.interface.ext_mem.External_Mem_DCR_FlightTask_1 + offset, PART_DCR_FLIGHT_TASK_1);
      }
      else if((id.uf.Offset >= 128*4) && (id.uf.Offset < 128*5)){
        ext_mem_read_from_part_8b(&lm.mem, offset, lm.interface.ext_mem.External_Mem_DCR_FlightTask_2 + offset, PART_DCR_FLIGHT_TASK_2);
      }
      else if((id.uf.Offset >= 128*5) && (id.uf.Offset < 128*6)){
        ext_mem_read_from_part_8b(&lm.mem, offset, lm.interface.ext_mem.External_Mem_DCR_Status + offset, PART_DCR_STATUS);
      }
  }
}

/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  state: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackDCRInterface(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
	uint32_t i;
  if(state == 0) return;  //первый вызов тут не нужен
  if (id.std.RTR == 1) return;  //обработка чтения нам не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      dcr_inerface_process_cb(&lm.interface, i);
  }
}

/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  state: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackISSInterface(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
	uint32_t i;
  if(state == 0) return;  //первый вызов тут не нужен
  if (id.std.RTR == 1) return;  //обработка чтения нам не нужна
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<(id.uf.Offset + leng); i++){
      pl_iss_inerface_process_cb(&lm.interface, i);
  }
}

/**
  * @brief  регистрация колбэков для обработки команд CAN
  * @param  can_ptr: указатель на структуру управления CAN
  * @param  id: номер переменной для обработки CB
  * @param  leng: длина данных для обработки
  * @param  state: тип CB - 0 до записи данных, 1 - после
  */
void ProcCallbackCAN_Flash(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
  CallbackCAN_Flash(can_ptr, id, leng, state);
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
}

//*** Cmds process function ***//
/**
  * @brief  функция для создания псевдопотока для выполнения продолжительной команды: test_led
  *         - раз в 2 секунды увеличивает скважность на 2 бит (100% - 255 бит)
  *         - полный цикл 200 секунд: от отсутствия мигания до постоянного горения
  * @param  period_ms: период вызова обработчика в псевдопотоке
  */
void cmd_process_test_led(uint8_t mode, uint32_t period_ms)
{
  uint8_t cmd_code = CMD_DBG_LED_TEST;
  //*** Прерывание работы ***//
  if (mode == MODE_CANCEL){
      cmd_set_status(&lm.interface, cmd_code, CMD_STATUS_CANCEL);
      lm.cmd_ctrl[cmd_code].main_counter = 0;
      lm.cmd_ctrl[cmd_code].time_ms = 0;
      lm.cmd_ctrl[cmd_code].point_time_ms = 0;
      lm.cmd_ctrl[cmd_code].ena = 0;
      return;
    }
  else{
    //*** Запуск работы ***//
    if ((mode == MODE_START) && (lm.cmd_ctrl[cmd_code].ena == 0)){
      lm.cmd_ctrl[cmd_code].main_counter = 0;
      lm.cmd_ctrl[cmd_code].time_ms = 0;
      lm.cmd_ctrl[cmd_code].point_time_ms = 0;
      lm.cmd_ctrl[cmd_code].ena = 1;
      cmd_set_status(&lm.interface, cmd_code, CMD_STATUS_START);
      return;
    }
    //*** Собственно тело процесса ***//
    else if (lm.cmd_ctrl[cmd_code].ena == 1){
      lm.cmd_ctrl[cmd_code].time_ms += period_ms;
      lm.cmd_ctrl[cmd_code].point_time_ms += period_ms;
      if (lm.cmd_ctrl[cmd_code].point_time_ms >= 2000){
        lm.cmd_ctrl[cmd_code].point_time_ms = 0;
        lm.cmd_ctrl[cmd_code].main_counter += 2;
        led_alt_setup(&mcu_state_led, LED_BLINK, 500, (255 - lm.cmd_ctrl[cmd_code].main_counter & 0xFF), 2000);
      }
      if (lm.cmd_ctrl[cmd_code].main_counter >= 254){
        //
        lm.cmd_ctrl[cmd_code].ena = 2; // отправляем на окончание работы
      }
      cmd_set_status(&lm.interface, cmd_code, (lm.cmd_ctrl[cmd_code].main_counter >> 2) & 0x7F);
    }
    //*** Окончание работы ***//
    else if (lm.cmd_ctrl[cmd_code].ena == 2){
      lm.cmd_ctrl[cmd_code].main_counter = 0;
      lm.cmd_ctrl[cmd_code].time_ms = 0;
      lm.cmd_ctrl[cmd_code].point_time_ms = 0;
      lm.cmd_ctrl[cmd_code].ena = 0;
      cmd_set_status(&lm.interface, cmd_code, CMD_STATUS_FINISH);
    }
  }
}

/**
  * @brief  функция для копирования полетного задания ДеКоР из памяти CAN в програмную модель ДеКоР
  * @param  mode: команда, записанная в командный регистр
  * @param  cmd_code: номер полетного задания
  * @param  period_ms: период вызова обработчика в псевдопотоке
  */
void cmd_process_dcr_write_flight_task(uint8_t mode, uint8_t cmd_code, uint32_t period_ms)
{
  uint8_t *fligt_task_ptr=0, mem_part_dcr=0, fl_task_num = 1;
    //*** Прерывание работы ***//
  if (mode == MODE_CANCEL){
      cmd_set_status(&lm.interface, cmd_code, CMD_STATUS_CANCEL);
      lm.cmd_ctrl[cmd_code].main_counter = 0;
      lm.cmd_ctrl[cmd_code].time_ms = 0;
      lm.cmd_ctrl[cmd_code].point_time_ms = 0;
      lm.cmd_ctrl[cmd_code].ena = 0;
      return;
    }
  else{
    //*** Запуск работы ***//
    if ((mode == MODE_START) && (lm.cmd_ctrl[cmd_code].ena == 0)){
      lm.cmd_ctrl[cmd_code].main_counter = 0;
      lm.cmd_ctrl[cmd_code].time_ms = 0;
      lm.cmd_ctrl[cmd_code].point_time_ms = 0;
      lm.cmd_ctrl[cmd_code].ena = 1;
      cmd_set_status(&lm.interface, cmd_code, CMD_STATUS_START);
      return;
    }
    //*** Собственно тело процесса ***//
    else if (lm.cmd_ctrl[cmd_code].ena == 1){
      //
    switch(cmd_code){
      case CMD_DCR_WRITE_FLIGHT_TASK_1:
        fligt_task_ptr = (uint8_t*)lm.interface.dcr_interface.FlightTask_1;
        mem_part_dcr = PART_DCR_FLIGHT_TASK_1;
        fl_task_num = 1;
        break;
      case CMD_DCR_WRITE_FLIGHT_TASK_2:
        fligt_task_ptr = (uint8_t*)lm.interface.dcr_interface.FlightTask_2;
        mem_part_dcr = PART_DCR_FLIGHT_TASK_2;
        fl_task_num = 2;
        break;
      default:
        fligt_task_ptr = (uint8_t*)lm.interface.dcr_interface.FlightTask_1;
        mem_part_dcr = PART_DCR_FLIGHT_TASK_1;
        fl_task_num = 1;
        break;
    }
      pn_dcr_load_can_flight_task(&lm.pl._dcr, fligt_task_ptr, fl_task_num);
      for (uint8_t i=0; i<16; i++){
        ext_mem_wr_frame_from_part_by_addr(&lm.mem, fligt_task_ptr + i*128, i, mem_part_dcr);
      }
      //
      lm.cmd_ctrl[cmd_code].main_counter += 1;
      lm.cmd_ctrl[cmd_code].time_ms += period_ms;
      lm.cmd_ctrl[cmd_code].point_time_ms += period_ms;
      lm.cmd_ctrl[cmd_code].ena = 2; // отправляем на окончание работы
      cmd_set_status(&lm.interface, cmd_code, 2);
    }
    //*** Окончание работы ***//
    else if (lm.cmd_ctrl[cmd_code].ena == 2){
      lm.cmd_ctrl[cmd_code].main_counter = 0;
      lm.cmd_ctrl[cmd_code].time_ms = 0;
      lm.cmd_ctrl[cmd_code].point_time_ms = 0;
      lm.cmd_ctrl[cmd_code].ena = 0;
      cmd_set_status(&lm.interface, cmd_code, CMD_STATUS_FINISH);
    }
  }
}
