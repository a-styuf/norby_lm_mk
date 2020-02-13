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
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "vcp_time_segmentation.h"
#include "lm.h"
#include "led.h"
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
type_LM_DEVICE lm;
type_LED_INDICATOR mcu_state_led, con_state_led;

uint8_t tx_data[256], tx_data_len=0; //массив для формирования данных для отправки через VCP
uint8_t time_slot_flag = 0;
int8_t int8_val = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */
  lm_init(&lm);
  led_init(&mcu_state_led, GPIOD, 6);
  led_init(&con_state_led, GPIOD, 7);
  led_setup(&con_state_led, LED_OFF, 0, 0);
	led_setup(&mcu_state_led, LED_HEART_BEAT, 1000, 0);	
  //
  HAL_TIM_Base_Start_IT(&htim6); //LED timer
  HAL_TIM_Base_Start_IT(&htim2); //global clock timer
  HAL_TIM_Base_Start_IT(&htim3); //100ms time slot timer
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if (time_slot_flag){ // 100ms
			//опрос мониторов питания и температур
			pwr_process_100ms(&lm.pwr);
			//опрос тремодатчиков
			tmp_process_100ms(&lm.tmp);
			//работа с циклограммой
			int8_val = cyclogram_process_100ms(&lm.cyclogram, &lm.pl);
			if (int8_val > 0){
				led_alt_setup(&mcu_state_led, LED_BLINK, 300, 64, 1000);	
			}
			else if (int8_val < 0){
				led_alt_setup(&mcu_state_led, LED_BLINK, 300, 191, 1000);	
			}
			else{
			}
			//reset flag
			time_slot_flag = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// обработка команд USB-VCP
		if (vcp_uart_read(&vcp)){
			led_alt_setup(&con_state_led, LED_BLINK, 300, 127, 1000);	
			if (vcp.rx_buff[0] == DEV_ID){
				if (vcp.rx_buff[4] == 0x00){ //зеркало для ответа
					memcpy(tx_data, &vcp.rx_buff[6], vcp.rx_buff[5]&0x7F);
					tx_data_len = vcp.rx_buff[5];
					memcpy(tx_data, &vcp.rx_buff[6], vcp.tx_size);
				}
				else if (vcp.rx_buff[4] == 0x01){ //включение/отклюение каналов питания по выбору
					pwr_on_off(&lm.pwr, vcp.rx_buff[6]);
					tx_data_len = 0;
				}
				else if (vcp.rx_buff[4] == 0x02){ //ТМ�? системы питания
					memcpy(tx_data, &lm.pwr.report, sizeof(lm.pwr.report));
					tx_data_len = sizeof(lm.pwr.report);
				}
				else if (vcp.rx_buff[4] == 0x03){ //одиночный запуск циклограммы по выбору c выбранным режимом
					tx_data[0] = cyclogram_start(&lm.cyclogram, vcp.rx_buff[6], vcp.rx_buff[7]);
					tx_data_len = 1;
				}
				else if (vcp.rx_buff[4] == 0x04){ //состояние полезной нагрузки по выбору
					pl_report_get(&lm.pl, vcp.rx_buff[6], tx_data, &tx_data_len);
				}
				vcp.tx_size = com_ans_form(vcp.rx_buff[1], DEV_ID, &vcp.tx_seq_num, vcp.rx_buff[4], tx_data_len, tx_data, vcp.tx_buff);
				vcp_uart_write(&vcp, vcp.tx_buff, vcp.tx_size);
			}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim6) {
		led_processor(&mcu_state_led, 10);
		led_processor(&con_state_led, 10);
	}
	if (htim == &htim2) {
		lm.global_time_s += 1;
	}
	if (htim == &htim3) {
		time_slot_flag = 1;
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

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
