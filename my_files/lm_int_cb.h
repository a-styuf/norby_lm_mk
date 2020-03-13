#ifndef _LM_INT_CB_H
#define _LM_INT_CB_H

#include "lm.h"

void ProcCallbackCmds_Init(void);
void ProcCallbackCmds(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state);

#endif
