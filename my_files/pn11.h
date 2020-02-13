#ifndef _PN11_H
#define _PN11_H

#include "my_gpio.h"
#include "pwr_ch.h"
#include "tmp1075.h"

#define PN11_OUTPUT_DEFAULT 0x0F
#define PN11_OUTPUT_FPGA_ON 0x02
#define PN11_OUTPUT_FPGA_MCU_ON 0x00

/** 
  * @brief  структура хранения отчета по ПН1.1
  */
typedef struct
{
	uint16_t voltage;
	uint16_t current;
	uint16_t temp;
	uint8_t outputs, inputs;
} type_PN11_report;

/** 
  * @brief  структура хранения всех параметров ПН1.1 достаточных для управления
  */
typedef struct
{
	type_GPIO_setting input[4]; // 0-INT, 1-PWR_ERROR, 2-WATCHDOG, 3-CPU_ERROR
	type_GPIO_setting output[4]; // 0-RST_FPGA, 1-RST_LEON, 2-KU_2, 3-KU_3
	type_PWR_CHANNEL* pwr_ch;
	type_TMP1075_DEVICE* tmp_ch;
	uint16_t interrupt_timeout;
	type_PN11_report report;
} type_PN11_model;

void pn11_init(type_PN11_model* pn11_ptr, uint8_t num, type_PWR_CHANNEL* pwr_ch_ptr, type_TMP1075_DEVICE* tmp_ch_ptr);
void pn_11_output_set(type_PN11_model* pn11_ptr, uint8_t output_state);
void pn_11_report_create(type_PN11_model* pn11_ptr);
uint8_t pn_11_get_inputs_state(type_PN11_model* pn11_ptr);
uint8_t pn_11_get_outputs_state(type_PN11_model* pn11_ptr);
void pn_11_pwr_on(type_PN11_model* pn11_ptr);
void pn_11_pwr_off(type_PN11_model* pn11_ptr);

#endif
