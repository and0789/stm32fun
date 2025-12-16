/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include <stdio.h>
#include <string.h>

#include "usart.h"

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
/* USER CODE BEGIN Variables */

uint32_t ledRedCounter = 0;
uint32_t ledYellowCounter = 0;
uint32_t ledGreenCounter = 0;

char uart_buffer[100];

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledRedTask */
osThreadId_t ledRedTaskHandle;
const osThreadAttr_t ledRedTask_attributes = {
  .name = "ledRedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledYellowTask */
osThreadId_t ledYellowTaskHandle;
const osThreadAttr_t ledYellowTask_attributes = {
  .name = "ledYellowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledGreenTask */
osThreadId_t ledGreenTaskHandle;
const osThreadAttr_t ledGreenTask_attributes = {
  .name = "ledGreenTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartLedRedTask(void *argument);
void StartLedYellowTask(void *argument);
void StartLedGreenTask(void *argument);

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ledRedTask */
  ledRedTaskHandle = osThreadNew(StartLedRedTask, NULL, &ledRedTask_attributes);

  /* creation of ledYellowTask */
  ledYellowTaskHandle = osThreadNew(StartLedYellowTask, NULL, &ledYellowTask_attributes);

  /* creation of ledGreenTask */
  ledGreenTaskHandle = osThreadNew(StartLedGreenTask, NULL, &ledGreenTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  char msg[150];

  // Pesan startup
  sprintf(msg, "\r\n=== STM32H750 FreeRTOS LED Demo ===\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);

  /* Infinite loop */
  for(;;)
  {
    // Print status setiap 1 detik
    sprintf(msg, "Status - RED: %lu | YELLOW: %lu | GREEN: %lu | Tick: %lu\r\n",
            ledRedCounter, ledYellowCounter, ledGreenCounter, osKernelGetTickCount());
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), 1000);

    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartLedRedTask */
/**
* @brief Function implementing the ledRedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedRedTask */
void StartLedRedTask(void *argument)
{
  /* USER CODE BEGIN StartLedRedTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
    ledRedCounter++;

    // Print setiap 1000 kali toggle (agar tidak flood UART)
    if(ledRedCounter % 1000 == 0) {
      sprintf(uart_buffer, "LED RED: %lu toggles\r\n", ledRedCounter);
      HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), 100);
    }

    osDelay(500);
  }
  /* USER CODE END StartLedRedTask */
}

/* USER CODE BEGIN Header_StartLedYellowTask */
/**
* @brief Function implementing the ledYellowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedYellowTask */
void StartLedYellowTask(void *argument)
{
  /* USER CODE BEGIN StartLedYellowTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
    ledYellowCounter++;

    if(ledYellowCounter % 100 == 0) {
      sprintf(uart_buffer, "LED YELLOW: %lu toggles\r\n", ledYellowCounter);
      HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), 100);
    }

    osDelay(200);
  }
  /* USER CODE END StartLedYellowTask */
}

/* USER CODE BEGIN Header_StartLedGreenTask */
/**
* @brief Function implementing the ledGreenTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedGreenTask */
void StartLedGreenTask(void *argument)
{
  /* USER CODE BEGIN StartLedGreenTask */
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
    ledGreenCounter++;

    if(ledGreenCounter % 500 == 0) {
      sprintf(uart_buffer, "LED GREEN: %lu toggles\r\n", ledGreenCounter);
      HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, strlen(uart_buffer), 100);
    }

    osDelay(100);
  }
  /* USER CODE END StartLedGreenTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

