#ifndef _AA_CAN_H_
#define _AA_CAN_H_

#include <stm32f4xx.h>

#define SYSCLKFREQ  72000000

typedef union {
  struct {
    uint32_t res1 : 1;
    uint32_t RTR : 1;
    uint32_t IDE : 1;
    uint32_t EXID : 29;
    }std;
  struct {
    uint32_t res1 : 1;
    uint32_t RTR : 1;
    uint32_t res2 : 1;
    uint32_t Offset : 21;
    uint32_t VarId : 4;
    uint32_t DevId : 4;
    }uf;
}typeIdxMask;

#pragma pack(2)

typedef struct {
  void *VarPtr;
  uint32_t VarLeng;
  void (*CallBackProc)(CAN_TypeDef *can_ref, typeIdxMask id, uint16_t leng, int state);
  uint8_t ivar_id;
  uint8_t access_flgs;  //[0]=1 - read only; [1]=1 - regardless offset; [2]=1 - without filter
}typeRegistrationRec;

#define ERR_INVALID_PARAMS        -1
#define ERR_CAN_INIT_MODE         -2
#define ERR_CAN_NORMAL_MODE       -3
#define ERR_CAN_NO_FREE_FILTER    -4
#define ERR_CAN_NO_TXMAILBOXES    -5
#define ERR_CAN_ACCESS_RANGE      -7
#define ERR_CAN_DCL_INVALID       -8


int CAN_Init(CAN_TypeDef *can_ref);
int CAN_RegisterVar(int n, uint8_t dev_id);
int CAN_DeregisterVar(int filter_num);
int CAN_Tx(CAN_TypeDef *can_ref, typeIdxMask id, void *p_data, uint16_t leng);


#endif


