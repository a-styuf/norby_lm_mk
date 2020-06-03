#ifndef _CY15B104QN_SPI_H
#define _CY15B104QN_SPI_H

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "spi.h"
#include "my_gpio.h"
#include "debug.h"

// Раскрашивание переменных
#define CY15_VOLUME_BYTES (1 << 19)
//
#define CY15_WREN_OPCODE 0x06
#define CY15_WRITE_OPCODE 0x02
#define CY15_READ_OPCODE 0x03
#define CY15_FAST_READ_OPCODE 0x0B
#define CY15_RUID_OPCODE 0x4C
#define CY15_RDID_OPCODE 0x9F
#define CY15_RDSR_OPCODE 0x05
#define CY15_WRSN_OPCODE 0xC2
#define CY15_RDSN_OPCODE 0xC3

#define VALIDATION_BUFF_VAL_RD_ID {0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0xC2, 0x26, 0x08}
#define VALIDATION_SERIAL_NUMBER {0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x01, 0xAA}

#define ERROR_SPI  (0x01 << 0)
#define ERROR_ADDR (0x01 << 1)
#define ERROR_HAL (0x01 << 2)

/** 
  * @brief  структура хранения настроек и параметров SPI памяти CY15B104QN
  */
typedef struct
{
  SPI_HandleTypeDef* spi;
	type_GPIO_setting cs;
  uint8_t in_buff[256], out_buff[256];
  int8_t rx_tx_cmplt_flag;
  int8_t error; // 0: SPI-error, 1: addr-error, 2: HAL_Error
} type_CY15B104QN_CONTROL;

int8_t cy15_init(type_CY15B104QN_CONTROL* cy15_ptr, SPI_HandleTypeDef* spi_ptr, GPIO_TypeDef* cs_bank, uint16_t cs_pos);
int8_t cy15_write(type_CY15B104QN_CONTROL* cy15_ptr, uint32_t addr, uint8_t *buff, uint8_t len);
int8_t cy15_read(type_CY15B104QN_CONTROL* cy15_ptr, uint32_t addr, uint8_t *buff, uint8_t len);
int8_t cy15_read_dma(type_CY15B104QN_CONTROL* cy15_ptr, uint32_t addr,uint8_t *buff, uint8_t len);

void cy15_blocking_test(type_CY15B104QN_CONTROL* cy15_ptr);

#endif
