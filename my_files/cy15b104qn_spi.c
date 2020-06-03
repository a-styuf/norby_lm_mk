/**
  ******************************************************************************
  * @file           : cy15b104qn_spi.c
  * @version        : v1.0
  * @brief          : надстройка над CubeMX HAL для работы с внешней SPI памятью CY15B104QN
  * @author         : Стюф Алексей/Alexey Styuf <a-styuf@yandex.ru>
  ******************************************************************************
  */

#include "cy15b104qn_spi.h"
#include <stdio.h>

/**
  * @brief  инициализация работы с CY15X104QN: специально использую блокирующий режим (одна 128-битная транзакция ~10 мкс)
  * @param  cy15_ptr: структура для управления памятью
  * @retval статус ошибки: 1 - все хорошо, 0 - есть ошибка
  */
int8_t cy15_init(type_CY15B104QN_CONTROL* cy15_ptr, SPI_HandleTypeDef* spi_ptr, GPIO_TypeDef* cs_bank, uint16_t cs_pos)
{
  int8_t report = 0;
  uint8_t serial_number[8] = VALIDATION_SERIAL_NUMBER;
  //
  cy15_ptr->spi = spi_ptr;
  //cs-gpio: init_state: high, very high speed
  cy15_ptr->cs = gpio_parameters_set(cs_bank, cs_pos);
  memset(cy15_ptr->in_buff, 0x00, 256);
  memset(cy15_ptr->out_buff, 0x00, 256);
  cy15_ptr->error = 0;
  // чтение ID-номеров микросхемы (RDID)
  cy15_ptr->out_buff[0] = CY15_RDID_OPCODE;
  gpio_set(&cy15_ptr->cs, 0);
  HAL_SPI_TransmitReceive(spi_ptr, cy15_ptr->out_buff, cy15_ptr->in_buff, 1+9, 1);
  gpio_set(&cy15_ptr->cs, 1);
  // чтение статуса
  cy15_ptr->out_buff[0] = CY15_RDSR_OPCODE;
  gpio_set(&cy15_ptr->cs, 0);
  HAL_SPI_TransmitReceive(spi_ptr, cy15_ptr->out_buff, cy15_ptr->in_buff, 1+1, 1);
  gpio_set(&cy15_ptr->cs, 1);
  // запись/чтение серийного номера для проверки (WRSN/RDSN)
  cy15_ptr->out_buff[0] = CY15_WREN_OPCODE;
  gpio_set(&cy15_ptr->cs, 0);
  HAL_SPI_Transmit(spi_ptr, cy15_ptr->out_buff, 1, 1);
  gpio_set(&cy15_ptr->cs, 1);
  //
  cy15_ptr->out_buff[0] = CY15_WRSN_OPCODE;
  memcpy(&cy15_ptr->out_buff[1], serial_number, sizeof(serial_number));
  gpio_set(&cy15_ptr->cs, 0);
  HAL_SPI_Transmit(spi_ptr, cy15_ptr->out_buff, 1+sizeof(serial_number), 1);
  gpio_set(&cy15_ptr->cs, 1);
  //
  memset(cy15_ptr->out_buff, 0x00, 256);
  cy15_ptr->out_buff[0] = CY15_RDSN_OPCODE;
  gpio_set(&cy15_ptr->cs, 0);
  HAL_SPI_TransmitReceive(spi_ptr, cy15_ptr->out_buff, cy15_ptr->in_buff, 1+sizeof(serial_number), 2);
  gpio_set(&cy15_ptr->cs, 1);
  //
  // printf_buff(serial_number, 8, '\t');
  // printf_buff(cy15_ptr->in_buff+1, 8, '\n');
  if (memcmp(serial_number, cy15_ptr->in_buff+1, sizeof(serial_number)) == 0){
    report += 1;
  }
  //
  return report;
}

/**
  * @brief  запись массива в память
  * @param  cy15_ptr: структура для управления памятью
  * @param  buff: указатель на блок памяти
  * @param  len: длина данных (максимум 128 байт)
  * @retval 1 - запись прошла успешно, 0 - ошибка
  */
int8_t cy15_write(type_CY15B104QN_CONTROL* cy15_ptr, uint32_t addr, uint8_t *buff, uint8_t len)
{
  int8_t report = 1;
  if ((addr + len) > CY15_VOLUME_BYTES) {
    report = 0;
    cy15_ptr->error |= ERROR_ADDR;
    return report;
  }
  //
  gpio_set(&cy15_ptr->cs, 0);
  cy15_ptr->out_buff[0] = CY15_WREN_OPCODE;
  if (HAL_SPI_Transmit(cy15_ptr->spi, cy15_ptr->out_buff, 1, 1) != HAL_OK) report = 0;
  else cy15_ptr->error |=ERROR_SPI;
  gpio_set(&cy15_ptr->cs, 1);
  //
  cy15_ptr->out_buff[0] = CY15_WRITE_OPCODE;
  cy15_ptr->out_buff[1] = (addr >> 16) & 0xFF;
  cy15_ptr->out_buff[2] = (addr >> 8) & 0xFF;
  cy15_ptr->out_buff[3] = (addr >> 0) & 0xFF;
  memcpy(&cy15_ptr->out_buff[4], buff, len);
  gpio_set(&cy15_ptr->cs, 0);
  if (HAL_SPI_Transmit(cy15_ptr->spi, cy15_ptr->out_buff, 4+len, 2) != HAL_OK) report = 0;
  else cy15_ptr->error |=ERROR_SPI;
  gpio_set(&cy15_ptr->cs, 1);
  //
  return report;
}

/**
  * @brief  чтение массива из памяти
  * @param  cy15_ptr: структура для управления памятью
  * @param  buff: указатель на блок памяти
  * @param  len: длина данных (максимум 128 байт)
  * @retval 1 - запись прошла успешно, 0 - ошибка
  */
int8_t cy15_read(type_CY15B104QN_CONTROL* cy15_ptr, uint32_t addr,uint8_t *buff, uint8_t len)
{
  int8_t report = 1;
  if ((addr + len) > CY15_VOLUME_BYTES) {
    report = 0;
    cy15_ptr->error |= ERROR_ADDR;
    return report;
  }
  memset(cy15_ptr->out_buff, 0x00, 256);
  cy15_ptr->out_buff[0] = CY15_FAST_READ_OPCODE;
  cy15_ptr->out_buff[1] = (addr >> 16) & 0xFF;
  cy15_ptr->out_buff[2] = (addr >> 8) & 0xFF;
  cy15_ptr->out_buff[3] = (addr >> 0) & 0xFF;
  cy15_ptr->out_buff[4] = 0x00;
  //
  gpio_set(&cy15_ptr->cs, 0);
  
  if (HAL_SPI_TransmitReceive(cy15_ptr->spi, cy15_ptr->out_buff, cy15_ptr->in_buff, 5+len, 2) != HAL_OK) report = 0;
  else cy15_ptr->error |= ERROR_SPI;
  gpio_set(&cy15_ptr->cs, 1);
  //  
  memcpy(buff, cy15_ptr->in_buff+5, len);
  return report;
}
