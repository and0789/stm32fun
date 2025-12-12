/**
 * @file audio_crypto.h
 * @brief Audio Encryption/Decryption using STM32H750 Hardware CRYP
 * @author Jerry
 * @date 2024
 * @version 2.0 - Fixed alignment issues
 */

#ifndef AUDIO_CRYPTO_H
#define AUDIO_CRYPTO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * CONFIGURATION
 * ============================================================================ */

/** Audio buffer size (samples per channel) */
#define AUDIO_BUFFER_SIZE       256

/** Number of audio channels (stereo = 2) */
#define AUDIO_CHANNELS          2

/** Total buffer size in 32-bit words */
#define AUDIO_DMA_BUFFER_SIZE   (AUDIO_BUFFER_SIZE * AUDIO_CHANNELS)

/** AES block size in bytes */
#define AES_BLOCK_SIZE          16

/** AES key size in 32-bit words (128-bit = 4 words) */
#define AES_KEY_SIZE_WORDS      4

/* ============================================================================
 * ENUMERATIONS
 * ============================================================================ */

/**
 * @brief Audio processing mode
 */
typedef enum {
    AUDIO_MODE_BYPASS = 0,          /**< Pass-through (no encryption) */
    AUDIO_MODE_ENCRYPT,             /**< Encrypt only (for transmission) */
    AUDIO_MODE_ENCRYPT_DECRYPT,     /**< Encrypt then decrypt (loopback test) */
    AUDIO_MODE_COUNT                /**< Number of modes */
} AudioMode_t;

/**
 * @brief Audio processing state
 */
typedef enum {
    AUDIO_STATE_IDLE = 0,
    AUDIO_STATE_RUNNING,
    AUDIO_STATE_ERROR
} AudioState_t;

/**
 * @brief DMA buffer position indicator
 */
typedef enum {
    BUFFER_HALF_FIRST = 0,
    BUFFER_HALF_SECOND
} BufferHalf_t;

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Audio crypto context structure
 * @note All buffers must be 4-byte aligned for DMA and CRYP
 */
typedef struct {
    AudioMode_t     mode;               /**< Current processing mode */
    AudioState_t    state;              /**< Current state */
    
    /* DMA Buffers - must be aligned for DMA */
    uint32_t        rxBuffer[AUDIO_DMA_BUFFER_SIZE];    /**< SAI RX buffer */
    uint32_t        txBuffer[AUDIO_DMA_BUFFER_SIZE];    /**< SAI TX buffer */
    uint32_t        cryptBuffer[AUDIO_DMA_BUFFER_SIZE]; /**< Crypto work buffer */
    
    /* AES Key and IV - uint32_t for proper alignment */
    uint32_t        aesKey[AES_KEY_SIZE_WORDS];     /**< AES-128 key (4 words) */
    uint32_t        aesIV[AES_KEY_SIZE_WORDS];      /**< AES-CTR initial vector (4 words) */
    uint32_t        ctrCounter;                      /**< CTR mode counter */
    
    /* Processing flags */
    volatile bool   rxHalfComplete;     /**< RX first half ready */
    volatile bool   rxFullComplete;     /**< RX second half ready */
    volatile bool   txHalfComplete;     /**< TX first half ready */
    volatile bool   txFullComplete;     /**< TX second half ready */
    
    /* Statistics */
    uint32_t        processedFrames;    /**< Total frames processed */
    uint32_t        encryptedFrames;    /**< Total frames encrypted */
    uint32_t        errorCount;         /**< Error counter */
    
} AudioCrypto_Context_t;

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * @brief Initialize audio crypto system
 * @param hsai_tx Pointer to SAI TX handle (PCM5102A)
 * @param hsai_rx Pointer to SAI RX handle (INMP441)
 * @param hcryp Pointer to CRYP handle
 * @return HAL_OK on success
 */
HAL_StatusTypeDef AudioCrypto_Init(SAI_HandleTypeDef *hsai_tx, 
                                    SAI_HandleTypeDef *hsai_rx,
                                    CRYP_HandleTypeDef *hcryp);

/**
 * @brief Start audio streaming
 * @return HAL_OK on success
 */
HAL_StatusTypeDef AudioCrypto_Start(void);

/**
 * @brief Stop audio streaming
 * @return HAL_OK on success
 */
HAL_StatusTypeDef AudioCrypto_Stop(void);

/**
 * @brief Set audio processing mode
 * @param mode New processing mode
 */
void AudioCrypto_SetMode(AudioMode_t mode);

/**
 * @brief Get current audio processing mode
 * @return Current mode
 */
AudioMode_t AudioCrypto_GetMode(void);

/**
 * @brief Cycle to next mode (for button press)
 */
void AudioCrypto_NextMode(void);

/**
 * @brief Get mode name string
 * @param mode Mode to get name for
 * @return Pointer to mode name string
 */
const char* AudioCrypto_GetModeName(AudioMode_t mode);

/**
 * @brief Set AES key
 * @param key Pointer to 4-word (128-bit) key array
 */
void AudioCrypto_SetKey(const uint32_t *key);

/**
 * @brief Set AES Initial Vector
 * @param iv Pointer to 4-word (128-bit) IV array
 */
void AudioCrypto_SetIV(const uint32_t *iv);

/**
 * @brief Process audio buffer (called from DMA callback)
 * @param bufferHalf Which half of buffer to process
 */
void AudioCrypto_ProcessBuffer(BufferHalf_t bufferHalf);

/**
 * @brief Get processing statistics
 * @param processed Pointer to store processed frame count
 * @param encrypted Pointer to store encrypted frame count
 * @param errors Pointer to store error count
 */
void AudioCrypto_GetStats(uint32_t *processed, uint32_t *encrypted, uint32_t *errors);

/**
 * @brief Reset statistics
 */
void AudioCrypto_ResetStats(void);

/* ============================================================================
 * CALLBACK PROTOTYPES (to be called from stm32h7xx_it.c or main.c)
 * ============================================================================ */

/**
 * @brief SAI RX Half Complete callback
 */
void AudioCrypto_RxHalfCpltCallback(void);

/**
 * @brief SAI RX Full Complete callback
 */
void AudioCrypto_RxCpltCallback(void);

/**
 * @brief SAI TX Half Complete callback
 */
void AudioCrypto_TxHalfCpltCallback(void);

/**
 * @brief SAI TX Full Complete callback
 */
void AudioCrypto_TxCpltCallback(void);

/**
 * @brief Button press callback (mode change)
 */
void AudioCrypto_ButtonCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* AUDIO_CRYPTO_H */
