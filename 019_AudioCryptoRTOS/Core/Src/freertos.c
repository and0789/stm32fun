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

#include "audio_crypto_rtos.h"
#include <stdio.h>

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

/* Peripheral handles (akan di-set dari main.c) */
extern SAI_HandleTypeDef hsai_BlockA1;
extern SAI_HandleTypeDef hsai_BlockB1;
extern CRYP_HandleTypeDef hcryp;

/* Button debounce */
static uint32_t lastButtonTick = 0;
#define BUTTON_DEBOUNCE_MS  200

/* USER CODE END Variables */
/* Definitions for StatsTask */
osThreadId_t StatsTaskHandle;
const osThreadAttr_t StatsTask_attributes = {
    .name = "StatsTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for AudioProcessTas */
osThreadId_t AudioProcessTasHandle;
const osThreadAttr_t AudioProcessTas_attributes = {
    .name = "AudioProcessTas",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for ButtonTask */
osThreadId_t ButtonTaskHandle;
const osThreadAttr_t ButtonTask_attributes = {
    .name = "ButtonTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for audioQueue */
osMessageQueueId_t audioQueueHandle;
const osMessageQueueAttr_t audioQueue_attributes = {
    .name = "audioQueue"
};
/* Definitions for audioMutex */
osMutexId_t audioMutexHandle;
const osMutexAttr_t audioMutex_attributes = {
    .name = "audioMutex"
};
/* Definitions for audioSemaphore */
osSemaphoreId_t audioSemaphoreHandle;
const osSemaphoreAttr_t audioSemaphore_attributes = {
    .name = "audioSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* Fungsi untuk update LED berdasarkan Mode */
void App_UpdateLEDs(AudioMode_t mode)
{
    /* 1. Matikan semua LED dulu (Reset) */
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

    /* 2. Nyalakan salah satu sesuai Mode */
    switch (mode)
    {
    case AUDIO_MODE_BYPASS:
        /* Merah Nyala */
        HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
        break;

    case AUDIO_MODE_ENCRYPT:
        /* Kuning Nyala */
        HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_SET);
        break;

    case AUDIO_MODE_ENCRYPT_DECRYPT:
        /* Hijau Nyala */
        HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET);
        break;

    default:
        break;
    }
}

/* USER CODE END FunctionPrototypes */

void StartStatsTask(void* argument);
void StartAudioProcessTask(void* argument);
void StartButtonTask(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */
    /* Create the mutex(es) */
    /* creation of audioMutex */
    audioMutexHandle = osMutexNew(&audioMutex_attributes);

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* Create the semaphores(s) */
    /* creation of audioSemaphore */
    audioSemaphoreHandle = osSemaphoreNew(1, 1, &audioSemaphore_attributes);

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* Create the queue(s) */
    /* creation of audioQueue */
    audioQueueHandle = osMessageQueueNew(16, sizeof(uint16_t), &audioQueue_attributes);

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* creation of StatsTask */
    StatsTaskHandle = osThreadNew(StartStatsTask, NULL, &StatsTask_attributes);

    /* creation of AudioProcessTas */
    AudioProcessTasHandle = osThreadNew(StartAudioProcessTask, NULL, &AudioProcessTas_attributes);

    /* creation of ButtonTask */
    ButtonTaskHandle = osThreadNew(StartButtonTask, NULL, &ButtonTask_attributes);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
    /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartStatsTask */
/**
  * @brief  Function implementing the StatsTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartStatsTask */
void StartStatsTask(void* argument)
{
    /* USER CODE BEGIN StartStatsTask */
    uint32_t processed, encrypted, errors;

    printf("[Task] StatsTask started\r\n");

    /* Infinite loop - print stats every 5 seconds */
    for (;;)
    {
        /* Wait 5 seconds */
        osDelay(5000);

        /* Get statistics */
        AudioCrypto_GetStats(&processed, &encrypted, &errors);

        /* Print stats */
        printf("[STATS] Frames: %lu | Encrypted: %lu | Errors: %lu | Mode: %s\r\n",
               processed, encrypted, errors,
               AudioCrypto_GetModeName(AudioCrypto_GetMode()));
    }
    /* USER CODE END StartStatsTask */
}

/* USER CODE BEGIN Header_StartAudioProcessTask */
/**
* @brief Function implementing the AudioProcessTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioProcessTask */
void StartAudioProcessTask(void* argument)
{
    /* USER CODE BEGIN StartAudioProcessTask */
    uint32_t bufferHalf;
    osStatus_t status;

    printf("[Task] AudioProcessTask started\r\n");

    /* Wait a bit for system to stabilize */
    osDelay(100);

    /* Initialize Audio Crypto */
    if (AudioCrypto_Init(&hsai_BlockA1, &hsai_BlockB1, &hcryp) != HAL_OK)
    {
        printf("[ERROR] AudioCrypto_Init failed!\r\n");
        /* Suspend this task on error */
        osThreadSuspend(NULL);
    }

    /* Start audio streaming */
    if (AudioCrypto_Start() != HAL_OK)
    {
        printf("[ERROR] AudioCrypto_Start failed!\r\n");
        osThreadSuspend(NULL);
    }

    printf("[Task] Audio streaming started, waiting for DMA callbacks...\r\n");

    /* Infinite loop - wait for queue messages from DMA callbacks */
    for (;;)
    {
        /* Wait for buffer half indicator from queue (blocking) */
        status = osMessageQueueGet(audioQueueHandle, &bufferHalf, NULL, osWaitForever);

        if (status == osOK)
        {
            /* Process the buffer half */
            AudioCrypto_ProcessBuffer((BufferHalf_t)bufferHalf);
        }
    }
    /* USER CODE END StartAudioProcessTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the ButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void* argument)
{
    /* USER CODE BEGIN StartButtonTask */
    static uint8_t lastButtonState = 1; /* Pull-up = idle high */
    uint8_t currentButtonState;
    uint32_t currentTick;

    printf("[Task] ButtonTask started\r\n");

    // Fungsi Button
    App_UpdateLEDs(AUDIO_MODE_BYPASS);

    /* Infinite loop - poll button every 50ms */
    for (;;)
    {
        /* Read button state (PC5) */
        currentButtonState = HAL_GPIO_ReadPin(BTN_MODE_GPIO_Port, BTN_MODE_Pin);

        /* Detect falling edge (button pressed) */
        if (lastButtonState == 1 && currentButtonState == 0)
        {
            currentTick = HAL_GetTick();

            /* Debounce check */
            if (currentTick - lastButtonTick > BUTTON_DEBOUNCE_MS)
            {
                /* Button pressed - change mode */
                AudioCrypto_NextMode();

                /* LED position update */
                App_UpdateLEDs(AudioCrypto_GetMode());

                lastButtonTick = currentTick;
            }
        }

        lastButtonState = currentButtonState;

        /* Polling interval: 50ms */
        osDelay(50);
    }
    /* USER CODE END StartButtonTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
