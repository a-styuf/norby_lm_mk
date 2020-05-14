#ifndef _CAN_VCP_H_
#define _CAN_VCP_H_

#include "vcp_time_segmentation.h"
#include "canv.h"

//** error defines
#define ERROR_NO_ERROR                0x00
#define ERROR_INCORRECT_PACKET_LEN   (0x01 << 0)
#define ERROR_INCORRECT_DATA_LEN     (0x01 << 1)
#define ERROR_VAR_ACCESS_RANGE       (0x01 << 2)
//
#define ERROR_UNKNOWN_ERROR          (0x01 << 15)

#pragma pack(2)
/** 
  * @brief  структура структура приемного или передающего пакета
  */

typedef struct {
  uint8_t ncan; //+0
  uint8_t res1; //+1
  typeIdxMask id; //+2
  uint16_t leng; //+6
  uint8_t data[8]; //+8
}typePacket; //16

/** 
  * @brief  структура хранения всех для can-vcp
  */
typedef struct  
{
	type_VCP_UART *vcp;
  CAN_TypeDef *can;
  typePacket rx_packet, tx_packet;
  uint16_t error_flag;
  uint16_t error_cnt;
}type_CAN_VCP;


void can_vcp_init(type_CAN_VCP* can_vcp_ptr, type_VCP_UART *vcp_ptr);
void can_vcp_packet_send(type_CAN_VCP* can_vcp_ptr);
uint8_t can_vcp_read_process(type_CAN_VCP* can_vcp_ptr);
void _can_vcp_error_collector(type_CAN_VCP* can_vcp_ptr, uint16_t error);
void CAN_VCP_RX_Handler(type_CAN_VCP *can_vcp_ptr);

#endif
