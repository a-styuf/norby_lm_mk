#ifndef _LM_INT_CB_H
#define _LM_INT_CB_H

#include "lm.h"

// раскрашивание переменных для обработчиков функций
#define MODE_IDLE 0x00
#define MODE_START 0x01
#define MODE_PROCESS 0x02
#define MODE_CANCEL 0x03

void ProcCallbackCmds_Init(void);
void ProcCallbackCmds(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);
void ProcCallbackCmdRegs(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);

#endif
