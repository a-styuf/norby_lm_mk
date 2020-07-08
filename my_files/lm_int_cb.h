#ifndef _LM_INT_CB_H
#define _LM_INT_CB_H

#include "lm.h"
#include "led.h"
#include "debug.h"

// раскрашивание переменных для обработчиков функций
#define MODE_START CMD_START
#define MODE_WORK 0x02
#define MODE_CANCEL CMD_CANCEL


// раскрашивание переменных для обработчиков команд
#define CMD_TEST_LED 0x00

void ProcCallbackCmds_Init(void);
void ProcCallbackCmds(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);
void ProcCallbackCmdRegs(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);
void ProcCallbackExtMems(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);
void ProcCallbackDCRInterface(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);
void ProcCallbackISSInterface(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);
void ProcCallbackCAN_Flash(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);

void cmd_process_test_led(uint8_t mode, uint32_t period_ms);
void cmd_process_dcr_write_flight_task(uint8_t mode, uint8_t cmd_code, uint32_t period_ms);

#endif
