/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>

#include "lm.h"
#include "lm_int_cb.h"
#include "can_vcp.h"
#include "vcp_time_segmentation.h"
#include "led.h"
#include "cy15b104qn_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
type_VCP_UART vcp;
type_CAN_VCP can_vcp;
type_LM_DEVICE lm;
type_LED_INDICATOR mcu_state_led, con_state_led;

uint8_t tx_data[256], tx_data_len=0; //масив для формирования данных для отправки через VCP
uint8_t rx_data[256], rx_data_len=0;
uint8_t time_slot_flag_100ms = 0, time_slot_flag_10ms = 0, time_slot_flag_5ms = 0, time_slot_flag_1s = 0;
uint8_t timer_slot_5ms_counter = 0;
uint8_t uint8_val = 0, uint8_buff[128] = {0};
uint16_t uint16_val = 0;
int16_t int16_val = 0;
int8_t int8_val = 0;
char str[128];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void blocking_test(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_DMA_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  lm_init(&lm);
  ProcCallbackCmds_Init();
  can_vcp_init(&can_vcp, &vcp);
  blocking_test();
  //
  lm_load_parameters(&lm);
  //led init
  led_init(&mcu_state_led, GPIOD, 6);
  led_init(&con_state_led, GPIOD, 7);
  led_setup(&con_state_led, LED_OFF, 0, 0);
  led_setup(&mcu_state_led, LED_HEART_BEAT, 1000, 0);
  led_alt_setup(&con_state_led, LED_BLINK, 200, 127, 2000);
  led_alt_setup(&mcu_state_led, LED_BLINK, 200, 127, 2000);
  //
  HAL_TIM_Base_Start_IT(&htim6); //LED and 10ms_slot timer
  HAL_TIM_Base_Start_IT(&htim2); //global clock timer
  HAL_TIM_Base_Start_IT(&htim3); //100ms time slot timer
  HAL_UART_Receive_IT(lm.pl._11A.interface.tr_lvl.huart, lm.pl._11A.interface.tr_lvl.rx_data, 1);
  HAL_UART_Receive_IT(lm.pl._11B.interface.tr_lvl.huart, lm.pl._11B.interface.tr_lvl.rx_data, 1);
  HAL_UART_Receive_IT(lm.pl._12.interface.tr_lvl.huart, lm.pl._12.interface.tr_lvl.rx_data, 1);
  HAL_UART_Receive_IT(lm.pl._20.interface.huart, lm.pl._20.interface.rx_data, 1);
  //
  __HAL_DBGMCU_FREEZE_IWDG();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    if (time_slot_flag_1s){ // 1s
      // сохраняем рабочие параметры
      lm_save_parameters(&lm);
      //reset flag
			time_slot_flag_1s = 0;
    }
		if (time_slot_flag_100ms){ // 100ms
      // сохраняем время в управляющую струткуру
      lm.ctrl.global_time_s = clock_get_time_s();
			//формирование кадров телеметрии
      fill_tmi_and_beacon(&lm);
      fill_gen_tmi(&lm);
			//опрос мониторов питания
			pwr_process(&lm.pwr, 100);
			//опрос тремодатчиков
			tmp_process(&lm.tmp, 100);
			//работа с циклограммой
			lm_cyclogram_process(&lm, 100);
      fill_pl_cyclogramm_result(&lm);
      // работа с продолжительными функциями запускаемые через переменную команд (0x02)
      cmd_process_test_led(MODE_WORK, 100);
      cmd_process_dcr_write_flight_task(MODE_WORK, CMD_DCR_WRITE_FLIGHT_TASK_1, 100);
      cmd_process_dcr_write_flight_task(MODE_WORK, CMD_DCR_WRITE_FLIGHT_TASK_2, 100);
      // работа с декор и полетным заданием для декор
      pn_dcr_process(&lm.pl._dcr, 100);
			//reset flag
			time_slot_flag_100ms = 0;
		}
		if (time_slot_flag_10ms){ // 10ms
      //сброс WDG
      HAL_IWDG_Refresh(&hiwdg);
      //обработка данных принятых от декор и последуещее заполнение памяти кадров ими
      pn_dcr_process_rx_frames(&lm.pl._dcr, 10);
      fill_dcr_rx_frame(&lm);
      //
      fill_pl_iss_last_frame(&lm);
			//reset flag
			time_slot_flag_10ms = 0;
		}
    if (time_slot_flag_5ms){ // 5ms
      //поддрежка работы программной модели ПН_�?СС (включая уровень приложения и транспортный уровень, а также процедуры вычитывания больших объемов данных)
			pn_11_process(&lm.pl._11A, 5);
      pn_11_process(&lm.pl._11B, 5);
      pn_12_process(&lm.pl._12, 5);
      pn_20_process(&lm.pl._20, 5);
			//reset flag
			time_slot_flag_5ms = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //*** USB-CAN
    //* обработка команд *//
    int16_val = cmd_check_to_process(&lm.interface);
    if (int16_val != CMD_NO_ONE) led_alt_setup(&con_state_led, LED_BLINK, 1400, 127, 2800);
    switch(int16_val){
      case CMD_INIT_LM:
        #ifdef DEBUG
          printf_time(); printf("cmd: init LM - start\n");
        #endif
        lm_reset_state(&lm);
        #ifdef DEBUG
          printf_time(); printf("cmd: init LM - stop\n");
        #endif
        break;
      case CMD_INIT_ISS_MEM:
        #ifdef DEBUG
          printf_time(); printf("cmd: init iss memory\n");
        #endif
        ext_mem_format_part(&lm.mem, PART_ISS);
        break;
      case CMD_INIT_DCR_MEM:
        #ifdef DEBUG
          printf_time(); printf("cmd: init decor memory\n");
        #endif
        ext_mem_format_part(&lm.mem, PART_DCR);
        break;
      case CMD_DCR_WRITE_FLIGHT_TASK_1:
        #ifdef DEBUG
          printf_time(); printf("cmd: write dcr flight task 1 from_can to dcr-model\n");
        #endif
        cmd_process_dcr_write_flight_task(lm.interface.cmd.array[CMD_DCR_WRITE_FLIGHT_TASK_1], CMD_DCR_WRITE_FLIGHT_TASK_1, 0);
        break;
      case CMD_DCR_WRITE_FLIGHT_TASK_2:
        #ifdef DEBUG
          printf_time(); printf("cmd: write dcr flight task 2 from_can to dcr-model\n");
        #endif
        cmd_process_dcr_write_flight_task(lm.interface.cmd.array[CMD_DCR_WRITE_FLIGHT_TASK_2], CMD_DCR_WRITE_FLIGHT_TASK_2, 0);
        break;
      case CMD_DBG_LED_TEST:
        #ifdef DEBUG
          printf_time(); printf("cmd: (dbg) led test 0x%02X\n", lm.interface.cmd.array[CMD_DBG_LED_TEST]);
        #endif
        cmd_process_test_led(lm.interface.cmd.array[CMD_DBG_LED_TEST], 0);
        break;
      case CMD_NO_ONE:
        // NULL;
        break;
      default: //если команда в резерве, то реагируем немедленным выполнением
        cmd_set_status(&lm.interface, int16_val, 0x7F);
        break;
    }
    //* обработка командных регистров *//
    int16_val = cmdreg_check_to_process(&lm.interface);
    if (int16_val != CMD_NO_ONE) led_alt_setup(&con_state_led, LED_BLINK, 1400, 127, 2800);
    switch(int16_val){
      case CMDREG_LM_MODE:
        #ifdef DEBUG
          printf_time(); printf("cmdreg: LM mode 0x%02X\n", lm.interface.cmdreg.array[CMDREG_LM_MODE]);
        #endif
        break;
      case CMDREG_PL_PWR_SW:
        pwr_on_off(&lm.pwr, lm.interface.cmdreg.array[CMDREG_PL_PWR_SW]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: PL pwr sw 0x%02X\n", lm.interface.cmdreg.array[CMDREG_PL_PWR_SW]);
				#endif
        break;
      case CMDREG_PL_INH_0:
      case CMDREG_PL_INH_1:
      case CMDREG_PL_INH_2:
      case CMDREG_PL_INH_3:
      case CMDREG_PL_INH_4:
      case CMDREG_PL_INH_5:
      case CMDREG_PL_INH_6:
      case CMDREG_PL_INH_7:
        #ifdef DEBUG
          printf_time(); printf("cmdreg: PL inhibit %d: 0x%02X\n", (int16_val - CMDREG_PL_INH_0), lm.interface.cmdreg.array[int16_val]);
        #endif
        lm_pl_inhibit_set(&lm, (int16_val - CMDREG_PL_INH_0), lm.interface.cmdreg.array[int16_val]);
        break;
      case CMDREG_DCR_MODE_SET:
        pn_dcr_set_mode(&lm.pl._dcr, lm.interface.cmdreg.array[CMDREG_DCR_MODE_SET]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: DCR set mode 0x%02X\n", lm.interface.cmdreg.array[CMDREG_DCR_MODE_SET]);
        #endif
        break;
      case CMDREG_PL11A_OUT_SET:
        pn_11_output_set(&lm.pl._11A, lm.interface.cmdreg.array[int16_val]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: PL11A set output 0x%02X\n", lm.interface.cmdreg.array[int16_val]);
        #endif
        break;  
      case CMDREG_PL11B_OUT_SET:
        pn_11_output_set(&lm.pl._11B, lm.interface.cmdreg.array[int16_val]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: PL11B set output 0x%02X\n", lm.interface.cmdreg.array[int16_val]);
        #endif
        break;  
      case CMDREG_PL12_OUT_SET:
        pn_12_output_set(&lm.pl._12, lm.interface.cmdreg.array[int16_val]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: PL12 set output 0x%02X\n", lm.interface.cmdreg.array[int16_val]);
        #endif
        break;  
      case CMDREG_PL20_OUT_SET:
        pn_20_output_set(&lm.pl._20, lm.interface.cmdreg.array[int16_val]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: PL20 set output 0x%02X\n", lm.interface.cmdreg.array[int16_val]);
        #endif
        break;  
			// case CMDREG_CYCLOGRAMS_0:
      case CMDREG_CYCLOGRAMS_1:
        #ifdef DEBUG
          printf_time(); printf("cmdreg: cyclograms 0x%04X\n", *(uint16_t*)&lm.interface.cmdreg.array[CMDREG_CYCLOGRAMS_0]);
        #endif
        cyclogram_start(&lm.cyclogram, &lm.pl, lm.interface.cmdreg.array[CMDREG_CYCLOGRAMS_0], lm.interface.cmdreg.array[CMDREG_CYCLOGRAMS_1]);
        break;
      case CMDREG_CONST_MODE:
        lm.ctrl.constant_mode = lm.interface.cmdreg.array[CMDREG_CONST_MODE] & 0x01;
        #ifdef DEBUG
          printf_time(); printf("cmdreg: const_mode 0x%02X\n", lm.interface.cmdreg.array[CMDREG_CONST_MODE]);
        #endif
        cyclogram_start(&lm.cyclogram, &lm.pl, lm.interface.cmdreg.array[CMDREG_CYCLOGRAMS_0], lm.interface.cmdreg.array[CMDREG_CYCLOGRAMS_1]);
        break;
      case CMDREG_TIME_3:
        clock_set_time_s(*(uint32_t*)&lm.interface.cmdreg.array[CMDREG_TIME_0]);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: synch time %d\n", *(uint32_t*)&lm.interface.cmdreg.array[CMDREG_TIME_0]);
        #endif
        break;
      case CMDREG_PART_MEM_RD_PTR_3:
        #ifdef DEBUG
          printf_time(); printf("cmdreg: set part %d rd_ptr to %d\n", lm.interface.cmdreg.array[CMDREG_PART_MEM_RD_PTR], 
                                                                      *(uint32_t*)&lm.interface.cmdreg.array[CMDREG_PART_MEM_RD_PTR_0]);
				#endif
				ext_mem_set_rd_ptr_for_part(&lm.mem, &lm.interface.cmdreg.array[CMDREG_PART_MEM_RD_PTR], 
                                      (uint32_t*)&lm.interface.cmdreg.array[CMDREG_PART_MEM_RD_PTR_0]);
        
        break;
      case CMDREG_SOFT_RESET:
        #ifdef DEBUG
          printf_time(); printf("cmdreg: soft reset key = %02X\n", lm.interface.cmdreg.array[CMDREG_SOFT_RESET]);
        #endif
        if (lm.interface.cmdreg.array[CMDREG_SOFT_RESET] == CMDREG_SOFT_RESET_VALUE){
          printf_time(); printf("BB, sweety! ^_^\n\n");
          while(1){};
        }
        else lm.interface.cmdreg.array[CMDREG_SOFT_RESET] = 0xFF;
        break;
      case CMDREG_DBG_LED:
        led_alt_setup(&mcu_state_led, LED_BLINK, 1000, lm.interface.cmdreg.array[CMDREG_DBG_LED], 3000);
        #ifdef DEBUG
          printf_time(); printf("cmdreg: (dbg) led test 0x%02X\n", lm.interface.cmdreg.array[CMDREG_DBG_LED]);
        #endif
        break;
      case CMD_NO_ONE:
      default:
        // NULL;
        break;
    }
    //* обработка команды для интерфейса к Декор *//
    int16_val = dcr_inerface_check_to_process(&lm.interface);
    if (int16_val != CMD_NO_ONE) led_alt_setup(&con_state_led, LED_BLINK, 1400, 127, 2800);
    switch(int16_val){
      case DCR_INTERFACE_INSTASEND_LENG_OFFSET:
        pn_dcr_uart_send(&lm.pl._dcr.uart, lm.interface.dcr_interface.InstaMessage, lm.interface.dcr_interface.InstaMessage[DCR_INTERFACE_INSTASEND_LENG_OFFSET]);
        printf("dcr_int:Instasend 0x%02X\n", lm.interface.dcr_interface.InstaMessage[DCR_INTERFACE_INSTASEND_LENG_OFFSET]);
        printf_buff(lm.interface.dcr_interface.InstaMessage, lm.interface.dcr_interface.InstaMessage[DCR_INTERFACE_INSTASEND_LENG_OFFSET], '\n');
        lm.interface.dcr_interface.InstaMessage[DCR_INTERFACE_INSTASEND_LENG_OFFSET] = 0;  // обнуляем для проверки отправки
        break;
      case CMD_NO_ONE:
      default:
        // NULL;
        break;
    }
    //* обработка команды для интерфейса к ПН_�?СС *//
    int16_val = pl_iss_inerface_check_to_process(&lm.interface);
    if (int16_val != CMD_NO_ONE) led_alt_setup(&con_state_led, LED_BLINK, 700, 150, 2800);
    switch(int16_val){
      case PL11A_INTERFACE_INSTASEND_LENG_OFFSET:
        pl_iss_get_app_lvl_reprot(PL11A, lm.interface.pl_iss_interface.InstaMessage[PL11A-1], str);
        printf("%s", str);
        pn_11_can_instasend(&lm.pl._11A, lm.interface.pl_iss_interface.InstaMessage[PL11A-1]);
        break;
      case PL11B_INTERFACE_INSTASEND_LENG_OFFSET:
        pl_iss_get_app_lvl_reprot(PL11B, lm.interface.pl_iss_interface.InstaMessage[PL11B-1], str);
        printf("%s", str);
        pn_11_can_instasend(&lm.pl._11B, lm.interface.pl_iss_interface.InstaMessage[PL11B-1]);
        break;
      case PL12_INTERFACE_INSTASEND_LENG_OFFSET:
        pl_iss_get_app_lvl_reprot(PL12, lm.interface.pl_iss_interface.InstaMessage[PL12-1], str);
        printf("%s", str);
        pn_12_can_instasend(&lm.pl._12, lm.interface.pl_iss_interface.InstaMessage[PL12-1]);
        break;
      case PL20_INTERFACE_INSTASEND_LENG_OFFSET:
        pl_iss_get_app_lvl_reprot(PL20, lm.interface.pl_iss_interface.InstaMessage[PL20-1], str);
        printf("%s", str);
        pn_20_instasend(&lm.pl._20, lm.interface.pl_iss_interface.InstaMessage[PL20-1]);
        break;
      case CMD_NO_ONE:
      default:
        // NULL;
        break;
    }
    //*** VCP-CAN. ***//
    //* обработка команд USB-VCP *//
    if (can_vcp_read_process(&can_vcp)) {
			led_alt_setup(&con_state_led, LED_BLINK, 700, 100, 2800);
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV10;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void blocking_test(void)
{
  // test
  printf_time();
  printf("Start blocking test\n");
	///*** test mem
	// printf("Full mem vol frame %d\n", FULL_MEM_VOL_FRAMES);
	// for(uint8_t i=0; i<PART_NUM; i++){
	// 	printf("\tPart=%d, Start=%d, stop=%d, vol=%d;\n", i, lm.mem.part[i].start_frame_num, lm.mem.part[i].finish_frame_num, lm.mem.part[i].full_frame_num);
	// }
	///*** test pn
  // // pn_11_a
  // printf("\tPL_11A\n");
  // pn_11_dbg_test(&lm.pl._11A);
  // // pn_11_b
  // printf("\tPL_11B\n");
  // pn_11_dbg_test(&lm.pl._11B);
  // // pn_12
  // printf("\tPL_12\n");
  // pn_12_dbg_test(&lm.pl._12);
  // // pn_20
  // printf("\tPL_20\n");
  // pn_20_dbg_test(&lm.pl._20);
  //
  // pn_11_dbg_tr_lvl_test(&lm.pl._11A);
  //
  // printf("MEM DMA test\n");
  // cy15_blocking_test(&lm.mem.cy15b104[0]);
  //
  printf_time();
  printf("Finish test\n\n");
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6) {
    timer_slot_5ms_counter += 1;
    if (timer_slot_5ms_counter >= 2){ //создание искусственного события c периодом 10 мс
      timer_slot_5ms_counter = 0;
      time_slot_flag_10ms = 1;
    }
    led_processor(&mcu_state_led, 5);
    led_processor(&con_state_led, 5);
		time_slot_flag_5ms = 5;
	}
	if (htim == &htim2) {
		lm.ctrl.global_time_s += 1;
    time_slot_flag_1s = 1;
	}
	if (htim == &htim3) {
		time_slot_flag_100ms = 1;
	}
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c3){
		pwr_cb_it_process(&lm.pwr, 0);
	}
	if(hi2c == &hi2c2){
		tmp_cb_it_process(&lm.tmp, 0);
	}
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c3){
		pwr_cb_it_process(&lm.pwr, 0);
	}
	if(hi2c == &hi2c2){
		tmp_cb_it_process(&lm.tmp, 0);
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c == &hi2c3){
		pwr_cb_it_process(&lm.pwr, 1);
	}
	if(hi2c == &hi2c2){
		tmp_cb_it_process(&lm.tmp, 1);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	pwr_alert_gd_it_process(&lm.pwr, GPIO_Pin);
	tmp_alert_it_process(&lm.tmp, GPIO_Pin);
	led_alt_setup(&mcu_state_led, 2, 600, 127, 10000);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart_req)
{
	if(huart_req == &huart2){ // PL1.1A
		rx_uart_data(&lm.pl._11A.interface.tr_lvl);
	}
	if(huart_req == &huart4){ // PL1.1B
		rx_uart_data(&lm.pl._11B.interface.tr_lvl);
	}
  if(huart_req == &huart1){ // PL1.2
		rx_uart_data(&lm.pl._12.interface.tr_lvl);
	}
  if(huart_req == &huart3){ // PL2.0
		pn_20_int_rx_huart_cb(&lm.pl._20);
	}
	if(huart_req == &huart6){ // PL_DCR
		pn_dcr_uart_rx_prcs_cb(&lm.pl._dcr.uart);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart_req)
{
	if(huart_req == &huart2){ // PL1.1A
		tr_lvl_set_timeout(&lm.pl._11A.interface.tr_lvl);
	}
	if(huart_req == &huart4){ // PL1.1B
		tr_lvl_set_timeout(&lm.pl._11B.interface.tr_lvl);
	}
  if(huart_req == &huart1){ // PL1.2
		tr_lvl_set_timeout(&lm.pl._12.interface.tr_lvl);
	}
  if(huart_req == &huart3){ // PL2.0
	  pn_20_int_tx_prcs_cb(&lm.pl._20);
	}
	if(huart_req == &huart6){ // PL_DCR
		pn_dcr_uart_tx_prcs_cb(&lm.pl._dcr.uart);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart_req)
{
  if(huart_req == &huart6){ // PL_DCR
		pn_dcr_uart_err_prcs_cb(&lm.pl._dcr.uart);
	}
  if(huart_req == &huart6){ // PL2.0
		pn_20_int_err_prcs_cb(&lm.pl._20);
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi2){
    lm.mem.cy15b104[0].rx_tx_cmplt_flag = 1;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi2){
    lm.mem.cy15b104[0].rx_tx_cmplt_flag = -1;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf("HAL error\n");
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
