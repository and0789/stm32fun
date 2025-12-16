/**
 * @file audio_crypto_rtos.h
 * @brief Audio Encryption/Decryption with FreeRTOS
 * @author Andre Septian
 * @date 2025
 */

#ifndef AUDIO_CRYPTO_RTOS_H
#define AUDIO_CRYPTO_RTOS_H

#include "stm32h7xx_hal.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * CONSTANTS & MACROS
 * ============================================================================ */

/* Audio Configuration */
#define AUDIO_SAMPLE_RATE       48000
#define AUDIO_CHANNELS          2
#define AUDIO_BITS_PER_SAMPLE   24
#define AUDIO_BUFFER_SIZE       4800  /* 50ms @ 48kHz stereo */
#define AUDIO_DMA_BUFFER_SIZE   (AUDIO_BUFFER_SIZE * 2)  /* Double buffer */

/* Buffer half indicator */
typedef enum {
    BUFFER_HALF_FIRST = 0,
    BUFFER_HALF_SECOND = 1
} BufferHalf_t;

/* Audio processing modes */
typedef enum {
    AUDIO_MODE_BYPASS = 0,           /* Pass-through */
    AUDIO_MODE_ENCRYPT = 1,          /* Encrypt only */
    AUDIO_MODE_ENCRYPT_DECRYPT = 2,  /* Encrypt then decrypt (loopback) */
    AUDIO_MODE_COUNT
} AudioMode_t;

/* Audio system state */
typedef enum {
    AUDIO_STATE_IDLE = 0,
    AUDIO_STATE_RUNNING,
    AUDIO_STATE_ERROR
} AudioState_t;

/* ============================================================================
 * STRUCTURES
 * ============================================================================ */

/**
 * @brief Audio crypto context
 */
typedef struct {
    /* Buffers - placed in D2 SRAM for DMA access */
    uint32_t rxBuffer[AUDIO_DMA_BUFFER_SIZE];  /* RX from mic */
    uint32_t txBuffer[AUDIO_DMA_BUFFER_SIZE];  /* TX to DAC */
    uint32_t cryptBuffer[AUDIO_BUFFER_SIZE];   /* Temp encryption buffer */

    /* AES Key and IV */
    uint32_t aesKey[4];   /* 128-bit key */
    uint32_t aesIV[4];    /* 128-bit IV */
    uint32_t ctrCounter;  /* CTR mode counter */

    /* State */
    AudioMode_t mode;
    AudioState_t state;

    /* Statistics */
    uint32_t processedFrames;
    uint32_t encryptedFrames;
    uint32_t errorCount;

} AudioCrypto_Context_t;

/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

/**
 * @brief Initialize audio crypto system
 * @param hsai_tx SAI TX handle (for DAC)
 * @param hsai_rx SAI RX handle (for MIC)
 * @param hcryp CRYP handle
 * @return HAL_OK if success
 */
HAL_StatusTypeDef AudioCrypto_Init(SAI_HandleTypeDef *hsai_tx,
                                    SAI_HandleTypeDef *hsai_rx,
                                    CRYP_HandleTypeDef *hcryp);

/**
 * @brief Start audio streaming
 * @return HAL_OK if success
 */
HAL_StatusTypeDef AudioCrypto_Start(void);

/**
 * @brief Stop audio streaming
 * @return HAL_OK if success
 */
HAL_StatusTypeDef AudioCrypto_Stop(void);

/**
 * @brief Process audio buffer (called by RTOS task)
 * @param bufferHalf Which half of buffer to process
 */
void AudioCrypto_ProcessBuffer(BufferHalf_t bufferHalf);

/**
 * @brief Set audio mode
 * @param mode New mode
 */
void AudioCrypto_SetMode(AudioMode_t mode);

/**
 * @brief Get current mode
 * @return Current mode
 */
AudioMode_t AudioCrypto_GetMode(void);

/**
 * @brief Cycle to next mode
 */
void AudioCrypto_NextMode(void);

/**
 * @brief Get mode name string
 * @param mode Mode to get name
 * @return Mode name string
 */
const char* AudioCrypto_GetModeName(AudioMode_t mode);

/**
 * @brief Get statistics
 * @param processed Pointer to store processed frames count
 * @param encrypted Pointer to store encrypted frames count
 * @param errors Pointer to store error count
 */
void AudioCrypto_GetStats(uint32_t *processed, uint32_t *encrypted, uint32_t *errors);

/**
 * @brief Reset statistics
 */
void AudioCrypto_ResetStats(void);

/**
 * @brief DMA RX Half Complete Callback (from ISR)
 */
void AudioCrypto_RxHalfCpltCallback(void);

/**
 * @brief DMA RX Complete Callback (from ISR)
 */
void AudioCrypto_RxCpltCallback(void);

/**
 * @brief DMA TX Half Complete Callback (from ISR)
 */
void AudioCrypto_TxHalfCpltCallback(void);

/**
 * @brief DMA TX Complete Callback (from ISR)
 */
void AudioCrypto_TxCpltCallback(void);

#endif /* AUDIO_CRYPTO_RTOS_H */