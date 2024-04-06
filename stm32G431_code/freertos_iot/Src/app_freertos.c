/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
u8 led__ctrl=0x00;
/* USER CODE END Variables */
/* Definitions for BodyInduction */
osThreadId_t BodyInductionHandle;
const osThreadAttr_t BodyInduction_attributes = {
  .name = "BodyInduction",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for lcd_debug */
osThreadId_t lcd_debugHandle;
uint32_t lcd_debugBuffer[ 512 ];
osStaticThreadDef_t lcd_debugControlBlock;
const osThreadAttr_t lcd_debug_attributes = {
  .name = "lcd_debug",
  .stack_mem = &lcd_debugBuffer[0],
  .stack_size = sizeof(lcd_debugBuffer),
  .cb_mem = &lcd_debugControlBlock,
  .cb_size = sizeof(lcd_debugControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for cntclear */
osThreadId_t cntclearHandle;
const osThreadAttr_t cntclear_attributes = {
  .name = "cntclear",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppBodyInductio(void *argument);
void StartLcd_debug(void *argument);
void Appcntclear(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BodyInduction */
  BodyInductionHandle = osThreadNew(AppBodyInductio, NULL, &BodyInduction_attributes);

  /* creation of lcd_debug */
  lcd_debugHandle = osThreadNew(StartLcd_debug, NULL, &lcd_debug_attributes);

  /* creation of cntclear */
  cntclearHandle = osThreadNew(Appcntclear, NULL, &cntclear_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_AppBodyInductio */
/**
  * @brief  Function implementing the BodyInduction thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_AppBodyInductio */
__weak void AppBodyInductio(void *argument)
{
  /* USER CODE BEGIN AppBodyInductio */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END AppBodyInductio */
}

/* USER CODE BEGIN Header_StartLcd_debug */
/**
* @brief Function implementing the lcd_debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLcd_debug */
__weak void StartLcd_debug(void *argument)
{
  /* USER CODE BEGIN StartLcd_debug */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartLcd_debug */
}

/* USER CODE BEGIN Header_Appcntclear */
/**
* @brief Function implementing the cntclear thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Appcntclear */
__weak void Appcntclear(void *argument)
{
  /* USER CODE BEGIN Appcntclear */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Appcntclear */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
