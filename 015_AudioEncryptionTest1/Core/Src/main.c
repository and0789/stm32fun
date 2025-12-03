/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "aes.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define ENCRYPTION_ENABLE  1  // 1 = Aktifkan Enkripsi (Noise), 0 = Suara Asli

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;

SD_HandleTypeDef hsd1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */


/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Audio Encryption dengan Buffer Terpisah
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "aes.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SAI_HandleTypeDef hsai_BlockA1;
SAI_HandleTypeDef hsai_BlockB1;
DMA_HandleTypeDef hdma_sai1_a;
DMA_HandleTypeDef hdma_sai1_b;
SD_HandleTypeDef hsd1;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */

#define AUDIO_BUFFER_SIZE 4096

// Buffer terpisah
ALIGN_32BYTES(int32_t rxBuffer[AUDIO_BUFFER_SIZE]) __attribute__((section(".RAM_D2")));
ALIGN_32BYTES(int32_t txBuffer[AUDIO_BUFFER_SIZE]) __attribute__((section(".RAM_D2")));

// AES Context
AES_ctx ctx_global;

const uint8_t key[16] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c
};
const uint8_t iv[16] = {
    0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
    0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff
};

// ✅ THRESHOLD - Sesuaikan dengan hardware Anda
#define NOISE_THRESHOLD 50000
#define MIN_LOUD_SAMPLES 5  // Minimal 5 sample loud untuk dianggap ada suara

// Flags
volatile uint8_t processHalf1 = 0;
volatile uint8_t processHalf2 = 0;

// Counters
volatile uint32_t tx_cplt_count = 0;
volatile uint32_t rx_cplt_count = 0;
volatile uint32_t err_count = 0;

// Status
volatile uint8_t encryption_active = 0;
volatile uint8_t gate_is_open = 0; // Track gate status

// DEBUG MODE: Set ke 1 untuk log detail sample
// PRODUCTION
// #define DEBUG_SAMPLE_MODE 0
// #define STATS_MODE 0

// SIMPLE TEST
// #define DEBUG_SAMPLE_MODE 0
// #define STATS_MODE 1

// DEEP TEST
// #define DEBUG_SAMPLE_MODE 1
// #define STATS_MODE 1

#define DEBUG_SAMPLE_MODE 0
#define DEBUG_SAMPLE_COUNT 100  // Jumlah sample yang di-log

// STATISTICS MODE: Set ke 1 untuk statistik real-time
#define STATS_MODE 0

#if STATS_MODE
typedef struct {
    uint32_t gate_open_count;
    uint32_t gate_shut_count;
    int32_t max_level;
    int32_t min_level;
    uint32_t samples_above_threshold;
    uint32_t total_samples_checked;
} AudioStats;

AudioStats stats = {0};
#endif

// Debounce (Agar tidak terpencet ganda)
uint32_t last_button_press = 0;

// Tambahan Flag Sinkronisasi
volatile uint8_t audio_tx_started = 0; // Penanda apakah speaker sudah nyala

// // Variable untuk SD Card
// ALIGN_32BYTES (FATFS fs) __attribute__((section(".RAM_D2")));
// FIL myFile;
// FRESULT res;
// UINT bytesWritten;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void PeriphCommonClock_Config(void);

static void MPU_Config(void);

static void MX_GPIO_Init(void);

static void MX_DMA_Init(void);

static void MX_SAI1_Init(void);

static void MX_USART1_UART_Init(void);

// static void MX_SDMMC1_SD_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int fd, char *ptr, int len) {
    if (fd == 1 || fd == 2) {
        HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, 100);
        return len;
    }
    return -1;
}

// Convert 24-bit signed ke int32_t
int32_t convert_24bit_to_32bit(int32_t raw) {
    if (raw & 0x00800000) {
        return (int32_t) (raw | 0xFF000000);
    } else {
        return (int32_t) (raw & 0x00FFFFFF);
    }
}

// ✅ DETEKSI AUDIO - Lebih ketat
uint8_t Is_Audio_Present(int32_t *buffer, uint32_t size) {
    uint32_t i;
    int32_t sample;
    int32_t abs_sample;
    uint32_t loud_samples = 0;

#if STATS_MODE
    stats.total_samples_checked += size;
#endif

    // Cek setiap 2 sample (lebih teliti)
    for (i = 0; i < size; i += 2) {
        sample = convert_24bit_to_32bit(buffer[i]);
        abs_sample = (sample < 0) ? -sample : sample;

#if STATS_MODE
        // Track min/max
        if (abs_sample > stats.max_level) stats.max_level = abs_sample;
        if (abs_sample < stats.min_level || stats.min_level == 0) stats.min_level = abs_sample;
#endif

        if (abs_sample > NOISE_THRESHOLD) {
            loud_samples++;

#if STATS_MODE
            stats.samples_above_threshold++;
#endif

            // Butuh minimal MIN_LOUD_SAMPLES untuk dianggap ada suara
            if (loud_samples >= MIN_LOUD_SAMPLES) {
                return 1;
            }
        }
    }

    return 0;
}

// ✅ PROCESS AUDIO - FIXED VERSION
void Process_Audio_Half(uint32_t half) {
    uint32_t offset = (half == 1) ? 0 : (AUDIO_BUFFER_SIZE / 2);
    uint32_t size = AUDIO_BUFFER_SIZE / 2;

    // 1. CEK APAKAH ADA SUARA
    uint8_t has_audio = Is_Audio_Present(&rxBuffer[offset], size);

    if (!has_audio) {
        // === HENING ===
        // Set output ke 0 (DIAM TOTAL)
        memset(&txBuffer[offset], 0, size * sizeof(int32_t));

#if STATS_MODE
        stats.gate_shut_count++;
#endif

        // ✅ KUNCI: Reset AES context saat gate tutup
        // Ini mencegah keystream terus berjalan saat tidak ada suara
        if (gate_is_open) {
            gate_is_open = 0;
            if (encryption_active) {
                AES_init_ctx_iv(&ctx_global, key, iv);
            }
        }

        return;
    }

    // === ADA SUARA ===
    gate_is_open = 1;

#if STATS_MODE
    stats.gate_open_count++;
#endif

    if (encryption_active) {
        // Mode Encrypted
        memcpy(&txBuffer[offset], &rxBuffer[offset], size * sizeof(int32_t));

        uint8_t *byteBuffer = (uint8_t *) &txBuffer[offset];
        uint32_t size_in_bytes = size * 4;

        AES_CTR_xcrypt_buffer(&ctx_global, byteBuffer, size_in_bytes);
    } else {
        // Mode Clear
        memcpy(&txBuffer[offset], &rxBuffer[offset], size * sizeof(int32_t));
    }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MPU Configuration--------------------------------------------------------*/
    MPU_Config();

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SAI1_Init();
    MX_USART1_UART_Init();
    // MX_SDMMC1_SD_Init();
    // MX_FATFS_Init();

    /* USER CODE BEGIN 2 */

    printf("\r\n=== AUDIO ENCRYPTION FINAL VERSION ===\r\n");
    printf("Noise Threshold: %d\r\n", NOISE_THRESHOLD);
    printf("Min Loud Samples: %d\r\n", MIN_LOUD_SAMPLES);

    AES_init_ctx_iv(&ctx_global, key, iv);
    printf("AES Initialized.\r\n");

    memset(rxBuffer, 0, sizeof(rxBuffer));
    memset(txBuffer, 0, sizeof(txBuffer));

    HAL_SAI_Receive_DMA(&hsai_BlockB1, (uint8_t *) rxBuffer, AUDIO_BUFFER_SIZE);
    HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *) txBuffer, AUDIO_BUFFER_SIZE);

    printf("Stream Started. Button PC5 = Toggle Encryption.\r\n\r\n");

    uint32_t last_stat_time = 0;
    uint32_t last_button_check = 0;

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        uint32_t now = HAL_GetTick();
        if (now - last_button_check > 200) {
            last_button_check = now;

            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
                encryption_active = !encryption_active;

                // Reset context saat toggle
                AES_init_ctx_iv(&ctx_global, key, iv);
                gate_is_open = 0;

                if (encryption_active)
                    printf("\r\n>>> ENKRIPSI ON <<<\r\n\r\n");
                else
                    printf("\r\n>>> ENKRIPSI OFF <<<\r\n\r\n");

                while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
                    HAL_Delay(10);
                }
            }
        }

        // Process audio
        if (processHalf1) {
            processHalf1 = 0;
            Process_Audio_Half(1);
        }

        if (processHalf2) {
            processHalf2 = 0;
            Process_Audio_Half(2);
        }

        // Monitoring
        if (HAL_GetTick() - last_stat_time > 1000) {
            last_stat_time = HAL_GetTick();

            int32_t sample = convert_24bit_to_32bit(rxBuffer[0]);
            int32_t level = (sample < 0) ? -sample : sample;

            printf("[%lu] TX:%lu RX:%lu | %s | Lvl:%ld | Gate:%s\r\n",
                   last_stat_time,
                   tx_cplt_count,
                   rx_cplt_count,
                   encryption_active ? "ENC" : "CLR",
                   level,
                   gate_is_open ? "OPEN" : "SHUT");

#if STATS_MODE
            // ✅ STATISTIK REAL-TIME
            float gate_open_ratio = (stats.gate_open_count + stats.gate_shut_count > 0)
                                        ? (100.0f * stats.gate_open_count / (
                                               stats.gate_open_count + stats.gate_shut_count))
                                        : 0;

            float threshold_ratio = (stats.total_samples_checked > 0)
                                        ? (100.0f * stats.samples_above_threshold / stats.total_samples_checked)
                                        : 0;

            printf("   Stats: Open=%lu Shut=%lu (%.1f%% open) | Max=%ld Min=%ld | Above_Th=%.2f%%\r\n",
                   stats.gate_open_count,
                   stats.gate_shut_count,
                   gate_open_ratio,
                   stats.max_level,
                   stats.min_level,
                   threshold_ratio);

            // Reset stats untuk periode berikutnya
            stats.gate_open_count = 0;
            stats.gate_shut_count = 0;
            stats.max_level = 0;
            stats.min_level = 0;
            stats.samples_above_threshold = 0;
            stats.total_samples_checked = 0;
#endif

#if DEBUG_SAMPLE_MODE
            // ✅ MODE DEBUG: Log 50 sample pertama untuk analisis
            printf("--- DEBUG: First %d samples ---\r\n", DEBUG_SAMPLE_COUNT);
            for (int i = 0; i < DEBUG_SAMPLE_COUNT; i++) {
                int32_t raw = rxBuffer[i];
                int32_t converted = convert_24bit_to_32bit(raw);
                int32_t tx_val = convert_24bit_to_32bit(txBuffer[i]);

                printf("[%d] RX_raw:%ld | RX_conv:%ld | TX:%ld\r\n",
                       i, raw, converted, tx_val);
            }
            printf("--- End of samples ---\r\n\r\n");
#endif
        }


        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Supply configuration update enable
    */
    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

    /** Configure the main internal regulator output voltage
    */
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 2;
    RCC_OscInitStruct.PLL.PLLN = 12;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 2;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                                  | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void) {
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /** Initializes the peripherals clock
    */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SAI1;
    PeriphClkInitStruct.PLL3.PLL3M = 5;
    PeriphClkInitStruct.PLL3.PLL3N = 49;
    PeriphClkInitStruct.PLL3.PLL3P = 20;
    PeriphClkInitStruct.PLL3.PLL3Q = 2;
    PeriphClkInitStruct.PLL3.PLL3R = 2;
    PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
    PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    PeriphClkInitStruct.PLL3.PLL3FRACN = 1245;
    PeriphClkInitStruct.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLL3;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        Error_Handler();
    }
}

/**
  * @brief SAI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI1_Init(void) {
    /* USER CODE BEGIN SAI1_Init 0 */

    /* USER CODE END SAI1_Init 0 */

    /* USER CODE BEGIN SAI1_Init 1 */

    /* USER CODE END SAI1_Init 1 */
    hsai_BlockA1.Instance = SAI1_Block_A;
    hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
    hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
    hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_ENABLE;
    hsai_BlockA1.Init.NoDivider = SAI_MCK_OVERSAMPLING_DISABLE;
    hsai_BlockA1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
    hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_48K;
    hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
    if (HAL_SAI_InitProtocol(&hsai_BlockA1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK) {
        Error_Handler();
    }
    hsai_BlockB1.Instance = SAI1_Block_B;
    hsai_BlockB1.Init.AudioMode = SAI_MODESLAVE_RX;
    hsai_BlockB1.Init.Synchro = SAI_SYNCHRONOUS;
    hsai_BlockB1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
    hsai_BlockB1.Init.MckOverSampling = SAI_MCK_OVERSAMPLING_DISABLE;
    hsai_BlockB1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
    hsai_BlockB1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
    hsai_BlockB1.Init.MonoStereoMode = SAI_STEREOMODE;
    hsai_BlockB1.Init.CompandingMode = SAI_NOCOMPANDING;
    hsai_BlockB1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
    if (HAL_SAI_InitProtocol(&hsai_BlockB1, SAI_I2S_STANDARD, SAI_PROTOCOL_DATASIZE_24BIT, 2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN SAI1_Init 2 */

    /* USER CODE END SAI1_Init 2 */
}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
// static void MX_SDMMC1_SD_Init(void)
// {
//
//   /* USER CODE BEGIN SDMMC1_Init 0 */
//
//   /* USER CODE END SDMMC1_Init 0 */
//
//   /* USER CODE BEGIN SDMMC1_Init 1 */
//
//   /* USER CODE END SDMMC1_Init 1 */
//   hsd1.Instance = SDMMC1;
//   hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
//   hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
//   hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
//   hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
//   hsd1.Init.ClockDiv = 6;
//   if (HAL_SD_Init(&hsd1) != HAL_OK)
//   {
//     Error_Handler();
//   }
//   /* USER CODE BEGIN SDMMC1_Init 2 */
//
//   /* USER CODE END SDMMC1_Init 2 */
//
// }

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void) {
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) {
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream0_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    /* DMA1_Stream1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    /* DMA1_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin : PC5 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai) {
    processHalf1 = 1;
}

void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai) {
    rx_cplt_count++;
    processHalf2 = 1;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai) {
    tx_cplt_count++;
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai) {
    err_count++;
}


// Konfigurasi MPU Manual
void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Matikan MPU dulu sebelum setting */
    HAL_MPU_Disable();

    // --- REGION 0: Default RAM (D1/AXI SRAM) ---
    // Cache AKTIF agar CPU kencang menjalankan program
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x24000000;
    MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    // --- REGION 1: RAM D2 (Audio Buffer) ---
    // Cache MATI (NOT_CACHEABLE) agar DMA dan CPU sinkron
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER1;
    MPU_InitStruct.BaseAddress = 0x30000000; // Alamat RAM D2
    MPU_InitStruct.Size = MPU_REGION_SIZE_256KB;
    MPU_InitStruct.SubRegionDisable = 0x0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE; // <--- KUNCI UTAMA!
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);

    /* Aktifkan MPU kembali */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/* USER CODE END 4 */

/* MPU Configuration */

// void MPU_Config(void)
// {
//   MPU_Region_InitTypeDef MPU_InitStruct = {0};
//
//   /* Disables the MPU */
//   HAL_MPU_Disable();
//
//   /** Initializes and configures the Region and the memory to be protected
//   */
//   MPU_InitStruct.Enable = MPU_REGION_ENABLE;
//   MPU_InitStruct.Number = MPU_REGION_NUMBER0;
//   MPU_InitStruct.BaseAddress = 0x0;
//   MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
//   MPU_InitStruct.SubRegionDisable = 0x87;
//   MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
//   MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
//   MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
//   MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
//   MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
//   MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
//
//   HAL_MPU_ConfigRegion(&MPU_InitStruct);
//   /* Enables the MPU */
//   HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
//
// }

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
