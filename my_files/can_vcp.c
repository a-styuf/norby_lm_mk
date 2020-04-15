/**
  ******************************************************************************
  * @file           : can_vcp.c
  * @version        : v1.0
  * @brief          :	позволяет работать с платой как с CAN_USB_bridge
  *						для корректной работы необходимы файлы vcp_time_segmentation./.h и aa_can.c/.h
  * @author			: Стюф Алексей/Alexe Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */
#include "can_vcp.h"

extern typeRegistrationRec RegistrationRec[14];

/**
  * @brief инициализация програмной модели для эмуляции работы чере CAN-USB-bridge
  */
void can_vcp_init(type_CAN_VCP* can_vcp_ptr, type_VCP_UART *vcp_ptr)
{
  // обнуляем переменные
  _can_vcp_error_collector(can_vcp_ptr, ERROR_NO_ERROR);
  //
  can_vcp_ptr->vcp = vcp_ptr;
}

/**
  * @brief отправка пакета на отправку
  * @param  can_vcp_ptr: структура управления приемом и передачей can-vcp
  * @note  
  */
void can_vcp_packet_send(type_CAN_VCP* can_vcp_ptr)
{
  can_vcp_ptr->vcp->tx_size = 8 + can_vcp_ptr->tx_packet.leng;
  memcpy(&can_vcp_ptr->vcp->tx_buff, &can_vcp_ptr->tx_packet, can_vcp_ptr->vcp->tx_size);
  vcp_uart_write(can_vcp_ptr->vcp, can_vcp_ptr->vcp->tx_buff, can_vcp_ptr->vcp->tx_size);
}

/**
  * @brief проверка на наличие данных принятых по vcp, подходящих под структуру typePacket
  * @param  can_vcp_ptr: структура управления приемом и передачей can-vcp
  */
uint8_t can_vcp_read_process(type_CAN_VCP* can_vcp_ptr)
{
  if (vcp_uart_read(can_vcp_ptr->vcp)){ // проверяем наличие данных на прием
    if (can_vcp_ptr->vcp->rx_size >= 8){ //проверяем количество данных на достоточность (8-байтовый заголовк typePacket)
      memcpy(&can_vcp_ptr->rx_packet, can_vcp_ptr->vcp->rx_buff, sizeof(typePacket));
      if ((can_vcp_ptr->rx_packet.ncan & 0x01) == 0) can_vcp_ptr->can = CAN1;
      else can_vcp_ptr->can = CAN2;
      CAN_VCP_RX_Handler(can_vcp_ptr);
      return(can_vcp_ptr->vcp->rx_size);
    }
    else {
      _can_vcp_error_collector(can_vcp_ptr, ERROR_INCORRECT_PACKET_LEN);
    }
  }
  else{
    return 0;
  }
  return 0;
}

/**
  * @brief сохранение и обработка ошибок в зависимости от их типа
  * @param  can_vcp_ptr: структура управления приемом и передачей can-vcp
  * @param  error: ошибка, согласно define-ом в .h
  */
void _can_vcp_error_collector(type_CAN_VCP* can_vcp_ptr, uint16_t error)
{
  switch(error){
    case ERROR_NO_ERROR:
      can_vcp_ptr->error_flag = ERROR_NO_ERROR;
      can_vcp_ptr->error_cnt = 0;
      break;
    case ERROR_INCORRECT_PACKET_LEN:
    case ERROR_INCORRECT_DATA_LEN:
    case ERROR_VAR_ACCESS_RANGE:
      can_vcp_ptr->error_flag |= error;
      can_vcp_ptr->error_cnt += 1;
      break;
    default:
      can_vcp_ptr->error_flag |= error;
      can_vcp_ptr->error_cnt += 1;
      break;
  }
}

/**
  * @brief модифицированный обработчик принятого пакета
  * @note  метка праки - correction (сокращение cr)
  * 
  */
void CAN_VCP_RX_Handler(type_CAN_VCP *can_vcp_ptr) 
{
  uint16_t i;
  uint16_t pkt_leng, filtr_num, bound; 
  typeIdxMask id; 
  uint8_t *pvar;
  int state = 0;
  id = can_vcp_ptr->rx_packet.id;    // cr: место структуры управления СAN привязался к своему интерфейсу
  pkt_leng = can_vcp_ptr->rx_packet.leng & 0x0F;    // cr: -- 
  filtr_num = id.uf.VarId;   // cr: --
  pvar = (uint8_t*)((uint32_t)RegistrationRec[filtr_num].VarPtr);
  bound = pkt_leng;
  if(pvar == (void*)0) {
    // can_ref->RF0R = 0x20;  //release FIFO        // cr: мне нет необходимости очищать фифо
    return;
    }
  if(pkt_leng > 8)
  {
    state = ERR_CAN_DCL_INVALID; 
    _can_vcp_error_collector(can_vcp_ptr, ERROR_INCORRECT_DATA_LEN);  // cr: свой сборщик ошибок
  }
  else {
    if((RegistrationRec[filtr_num].access_flgs & 0x02) == 0) { //offset is needed?
      pvar = pvar + id.uf.Offset;
      bound = bound + id.uf.Offset;
      }
    /* 1-st Callback */
    if(RegistrationRec[filtr_num].CallBackProc) {
      RegistrationRec[filtr_num].CallBackProc(can_vcp_ptr->can, id, pkt_leng, state); // cr: заменю на свой указатель на  can
      }
    if(id.std.RTR == 0) {  // запись
      if(bound <= RegistrationRec[filtr_num].VarLeng) {
        /*���������� �������*/
        for(i=0; i<pkt_leng; i++)
          pvar[i] = can_vcp_ptr->rx_packet.data[i];  // ((uint8_t*)&can_ref->sFIFOMailBox[0].RDLR)[i];  // cr: место структуры управления СAN привязался к своему интерфейсу
        }
      else {
        /*������������ �������*/
        state = ERR_CAN_ACCESS_RANGE;
        _can_vcp_error_collector(can_vcp_ptr, ERROR_VAR_ACCESS_RANGE);     // cr: свой сборщик ошибок
        }
      }
    else {  //RTR == 1 - // чтение
      id.std.RTR = 0;
      // state = CAN_Tx(can_ref, id, pvar, pkt_leng);  // cr: заменю на отправку в vcp
      memcpy(can_vcp_ptr->rx_packet.data, pvar, pkt_leng);
      memcpy(&can_vcp_ptr->tx_packet, &can_vcp_ptr->rx_packet, sizeof(typePacket));
      can_vcp_ptr->tx_packet.id.std.RTR = 0;
      can_vcp_packet_send(can_vcp_ptr);
      id.std.RTR = 1;  //restore RTR
      }
    }
  // can_ref->RF0R = 0x20;  //release FIFO        // cr: мне нет необходимости очищать фифо
  /* 2-nd Callback */
  if(RegistrationRec[filtr_num].CallBackProc) {
    if(state == 0) state = 1;
    RegistrationRec[filtr_num].CallBackProc(can_vcp_ptr->can, id, pkt_leng, state); // cr: заменю на свой указатель на  can
    }
}
