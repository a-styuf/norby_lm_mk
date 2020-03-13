/**
  ******************************************************************************
  * @file           : lm_int_cb.c
  * @version        : v1.0
  * @brief          : расширение для main, содержащие обрабюотчики для callback-ов интерфейсов МС
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "lm_int_cb.h"
#include <stdio.h>

extern type_LM_DEVICE lm;
//***  ***//

void ProcCallbackCmds_Init(void)
{
  interface_cb_registration(&lm.interface, ID_IVAR_CMD, ProcCallbackCmds);
}

//*** набор callback-функция для бработки команд интерфейса общения с внешним миром (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state); ***//

void ProcCallbackCmds(CAN_TypeDef *can_ptr, typeIdxMask id, uint16_t leng, int state) 
{
  volatile int n, vcmd;
	uint8_t i;
  if(state == 0)
    return;  //первый вызов тут не нужен
  if(can_ptr == CAN1) n = 1; else if(can_ptr == CAN2) n = 2; else n = 0;
	for (i=id.uf.Offset; i<CMD_CNT; i++){
		switch(i) {
			case 0: lm.interface.cmd_to_check[i] = lm.interface.cmd.CmdShort[i];  break;
			default: vcmd = -1;
    }
		lm.interface.cmd_flg = 1;
	}
  printf("CAN%d DevId=%d VarId=%d Offset=%d RTR=%d Leng=%d State=%d\n\r", n, id.uf.DevId, id.uf.VarId, id.uf.Offset, id.uf.RTR, leng, state);
}
