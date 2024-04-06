/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
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
u8 rx_buf[2];
u8 uart_buf[20];
char buffer[20];
u8 uart_cnt=0;
__IO uint32_t rxidle_tick=0;
//Wifi初始化为透传模式
void Wifi_Init(void)
{
	printf("AT+CIPSTART=\"TCP\",\"192.168.3.122\",8080\r\n");
	HAL_Delay(50);
	printf("AT+CIPMODE=1\r\n");
	HAL_Delay(50);
	printf("AT+CIPSEND\r\n");
}
//uart接收加尾数据检测
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_buf[uart_cnt++]=rx_buf[0];
	if(uart_buf[uart_cnt-1]=='a')
	{
		uart_cnt=0;
		HAL_UART_Transmit(&huart2,(uint8_t*)uart_buf,sizeof(uart_buf)-1,50);
		sprintf(buffer,"%20s",uart_buf);
		LCD_DisplayStringLine(Line8, (uint8_t *)buffer);
	}
	rxidle_tick=uwTick;
	HAL_UART_Receive_IT(&huart2,rx_buf,1);//重新开启串口中断，清除标志位与rx_buf,保证下一次的中断正常进行
}
//rx接收数据时间连续性判断
void rxidle_process(void)
{
	if(uwTick-rxidle_tick<50)
		return;
	uart_cnt=0;
	memset(uart_buf,'\0',sizeof(uart_buf));
}
//
void lcd_process(void)
{
	LCD_DisplayStringLine(Line0, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line1, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line2, (uint8_t *)"      LCD Test      ");
	LCD_DisplayStringLine(Line3, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line4, (uint8_t *)"                    ");	
	LCD_DisplayStringLine(Line5, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line6, (uint8_t *)"       HAL LIB      ");
	LCD_DisplayStringLine(Line7, (uint8_t *)"                    ");

	LCD_DisplayStringLine(Line9, (uint8_t *)"                    ");
}
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_USART2_UART_Init();
	
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
	Wifi_Init();
	//使能串口接收中断
	HAL_UART_Receive_IT(&huart2,rx_buf,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	LCD_DisplayStringLine(Line0, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line1, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line2, (uint8_t *)"      LCD Test      ");
	LCD_DisplayStringLine(Line3, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line4, (uint8_t *)"                    ");	
	LCD_DisplayStringLine(Line5, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line6, (uint8_t *)"       HAL LIB      ");
	LCD_DisplayStringLine(Line7, (uint8_t *)"                    ");
	LCD_DisplayStringLine(Line8, (uint8_t *)"         @80        ");
	LCD_DisplayStringLine(Line9, (uint8_t *)"                    ");
	
  while (1)
  {
    /* USER CODE END WHILE */
		rxidle_process();
		lcd_process();
    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
