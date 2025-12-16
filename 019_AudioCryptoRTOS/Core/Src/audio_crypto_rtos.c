/**
 * @file audio_crypto_rtos.c
 * @brief Audio Encryption/Decryption Implementation with FreeRTOS
 * @author Your Name
 * @date 2024
 */

#include "audio_crypto_rtos.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
 * EXTERNAL RTOS OBJECTS (defined in freertos.c)
 * ============================================================================ */

/* Import dari freertos.c */
extern osMessageQueueId_t audioQueueHandle;
extern osMutexId_t audioMutexHandle;
extern osThreadId_t AudioProcessTaskHandle;

/* ============================================================================
 * PRIVATE VARIABLES
 * ============================================================================ */

/** Audio crypto context - placed in D2 SRAM for DMA access */
static AudioCrypto_Context_t audioCtx __attribute__((section(".RAM_D2"), aligned(4)));

/** Peripheral handles */
static SAI_HandleTypeDef *hSaiTx = NULL;
static SAI_HandleTypeDef *hSaiRx = NULL;
static CRYP_HandleTypeDef *hCryp = NULL;

/** Default AES-128 Key - MUST be 4-byte aligned */
static const uint32_t DEFAULT_AES_KEY[4] __attribute__((aligned(4))) = {
    0x16157E2B, 0xA6D2AE28, 0x8815F7AB, 0x3C4FCF09
};

/** Default Initial Vector - MUST be 4-byte aligned */
static const uint32_t DEFAULT_AES_IV[4] __attribute__((aligned(4))) = {
    0x03020100, 0x07060504, 0x0B0A0908, 0x0F0E0D0C
};

/** Working IV buffer - 4-byte aligned for CRYP */
static uint32_t workingIV[4] __attribute__((aligned(4)));

/** Mode names for debug output */
static const char* MODE_NAMES[] = {
    "BYPASS",
    "ENCRYPT",
    "ENCRYPT+DECRYPT"
};

/* ============================================================================
 * PRIVATE FUNCTION PROTOTYPES
 * ============================================================================ */

static HAL_StatusTypeDef ProcessBypass(uint32_t *src, uint32_t *dst, uint32_t size);
static HAL_StatusTypeDef ProcessEncrypt(uint32_t *src, uint32_t *dst, uint32_t size);
static HAL_StatusTypeDef ProcessEncryptDecrypt(uint32_t *src, uint32_t *dst, uint32_t size);
static void UpdateCTRCounter(void);
static void PrepareIV(uint32_t counter);


/* ============================================================================
 * PUBLIC FUNCTIONS
 * ============================================================================ */

HAL_StatusTypeDef AudioCrypto_Init(SAI_HandleTypeDef *hsai_tx,
                                    SAI_HandleTypeDef *hsai_rx,
                                    CRYP_HandleTypeDef *hcryp)
{
    /* Validate parameters */
    if (hsai_tx == NULL || hsai_rx == NULL || hcryp == NULL) {
        return HAL_ERROR;
    }

    /* Store peripheral handles */
    hSaiTx = hsai_tx;
    hSaiRx = hsai_rx;
    hCryp = hcryp;

    /* Initialize context */
    memset(&audioCtx, 0, sizeof(AudioCrypto_Context_t));

    /* Set default mode */
    audioCtx.mode = AUDIO_MODE_BYPASS;
    audioCtx.state = AUDIO_STATE_IDLE;

    /* Set default key and IV */
    memcpy(audioCtx.aesKey, DEFAULT_AES_KEY, sizeof(DEFAULT_AES_KEY));
    memcpy(audioCtx.aesIV, DEFAULT_AES_IV, sizeof(DEFAULT_AES_IV));
    audioCtx.ctrCounter = 0;

    /* Clear buffers */
    memset(audioCtx.rxBuffer, 0, sizeof(audioCtx.rxBuffer));
    memset(audioCtx.txBuffer, 0, sizeof(audioCtx.txBuffer));
    memset(audioCtx.cryptBuffer, 0, sizeof(audioCtx.cryptBuffer));

    /* Initialize working IV */
    memset(workingIV, 0, sizeof(workingIV));

    printf("[AudioCrypto] Initialized - Mode: %s\r\n", MODE_NAMES[audioCtx.mode]);

    return HAL_OK;
}

HAL_StatusTypeDef AudioCrypto_Start(void)
{
    HAL_StatusTypeDef status;

    if (audioCtx.state == AUDIO_STATE_RUNNING) {
        return HAL_BUSY;
    }

    /* Pre-configure CRYP with default key */
    hCryp->Init.pKey = audioCtx.aesKey;
    hCryp->Init.pInitVect = workingIV;
    if (HAL_CRYP_Init(hCryp) != HAL_OK) {
        printf("[AudioCrypto] CRYP init failed\r\n");
        return HAL_ERROR;
    }

    /* Start SAI TX with DMA (DAC - PCM5102A) */
    status = HAL_SAI_Transmit_DMA(hSaiTx, (uint8_t*)audioCtx.txBuffer, AUDIO_DMA_BUFFER_SIZE);
    if (status != HAL_OK) {
        printf("[AudioCrypto] SAI TX DMA start failed: %d\r\n", status);
        return status;
    }

    /* Start SAI RX with DMA (MIC - INMP441) */
    status = HAL_SAI_Receive_DMA(hSaiRx, (uint8_t*)audioCtx.rxBuffer, AUDIO_DMA_BUFFER_SIZE);
    if (status != HAL_OK) {
        printf("[AudioCrypto] SAI RX DMA start failed: %d\r\n", status);
        HAL_SAI_DMAStop(hSaiTx);
        return status;
    }

    audioCtx.state = AUDIO_STATE_RUNNING;
    printf("[AudioCrypto] Audio streaming started\r\n");

    return HAL_OK;
}

HAL_StatusTypeDef AudioCrypto_Stop(void)
{
    HAL_StatusTypeDef status;

    /* Stop SAI RX */
    status = HAL_SAI_DMAStop(hSaiRx);
    if (status != HAL_OK) {
        printf("[AudioCrypto] SAI RX stop failed: %d\r\n", status);
    }

    /* Stop SAI TX */
    status = HAL_SAI_DMAStop(hSaiTx);
    if (status != HAL_OK) {
        printf("[AudioCrypto] SAI TX stop failed: %d\r\n", status);
    }

    audioCtx.state = AUDIO_STATE_IDLE;
    printf("[AudioCrypto] Audio streaming stopped\r\n");

    return HAL_OK;
}

void AudioCrypto_SetMode(AudioMode_t mode)
{
    if (mode < AUDIO_MODE_COUNT) {
        /* Lock mutex saat ubah mode */
        osMutexAcquire(audioMutexHandle, osWaitForever);

        audioCtx.mode = mode;
        audioCtx.ctrCounter = 0;  /* Reset counter on mode change */

        osMutexRelease(audioMutexHandle);

        printf("[AudioCrypto] Mode changed to: %s\r\n", MODE_NAMES[mode]);
    }
}

AudioMode_t AudioCrypto_GetMode(void)
{
    AudioMode_t mode;

    /* Lock mutex saat baca mode */
    osMutexAcquire(audioMutexHandle, osWaitForever);
    mode = audioCtx.mode;
    osMutexRelease(audioMutexHandle);

    return mode;
}

void AudioCrypto_NextMode(void)
{
    AudioMode_t newMode = (AudioCrypto_GetMode() + 1) % AUDIO_MODE_COUNT;
    AudioCrypto_SetMode(newMode);
}

const char* AudioCrypto_GetModeName(AudioMode_t mode)
{
    if (mode < AUDIO_MODE_COUNT) {
        return MODE_NAMES[mode];
    }
    return "UNKNOWN";
}

void AudioCrypto_GetStats(uint32_t *processed, uint32_t *encrypted, uint32_t *errors)
{
    if (processed) *processed = audioCtx.processedFrames;
    if (encrypted) *encrypted = audioCtx.encryptedFrames;
    if (errors) *errors = audioCtx.errorCount;
}

void AudioCrypto_ResetStats(void)
{
    audioCtx.processedFrames = 0;
    audioCtx.encryptedFrames = 0;
    audioCtx.errorCount = 0;
}

/* ============================================================================
 * BUFFER PROCESSING
 * ============================================================================ */

void AudioCrypto_ProcessBuffer(BufferHalf_t bufferHalf)
{
    uint32_t *srcPtr;
    uint32_t *dstPtr;
    uint32_t halfSize = AUDIO_DMA_BUFFER_SIZE / 2;
    HAL_StatusTypeDef status = HAL_OK;
    AudioMode_t currentMode;

    /* Determine buffer pointers based on which half to process */
    if (bufferHalf == BUFFER_HALF_FIRST) {
        srcPtr = &audioCtx.rxBuffer[0];
        dstPtr = &audioCtx.txBuffer[0];
    } else {
        srcPtr = &audioCtx.rxBuffer[halfSize];
        dstPtr = &audioCtx.txBuffer[halfSize];
    }

    /* Get current mode (thread-safe) */
    osMutexAcquire(audioMutexHandle, osWaitForever);
    currentMode = audioCtx.mode;
    osMutexRelease(audioMutexHandle);

    /* Process based on current mode */
    switch (currentMode) {
        case AUDIO_MODE_BYPASS:
            status = ProcessBypass(srcPtr, dstPtr, halfSize);
            break;

        case AUDIO_MODE_ENCRYPT:
            status = ProcessEncrypt(srcPtr, dstPtr, halfSize);
            if (status == HAL_OK) {
                audioCtx.encryptedFrames++;
            }
            break;

        case AUDIO_MODE_ENCRYPT_DECRYPT:
            status = ProcessEncryptDecrypt(srcPtr, dstPtr, halfSize);
            if (status == HAL_OK) {
                audioCtx.encryptedFrames++;
            }
            break;

        default:
            status = ProcessBypass(srcPtr, dstPtr, halfSize);
            break;
    }

    if (status != HAL_OK) {
        audioCtx.errorCount++;
    }

    audioCtx.processedFrames++;
}

/* ============================================================================
 * DMA CALLBACKS (Called from ISR context)
 * ============================================================================ */

void AudioCrypto_RxHalfCpltCallback(void)
{
    /* Send notification to AudioProcessTask via queue */
    uint32_t bufferHalf = BUFFER_HALF_FIRST;
    osMessageQueuePut(audioQueueHandle, &bufferHalf, 0, 0);
}

void AudioCrypto_RxCpltCallback(void)
{
    /* Send notification to AudioProcessTask via queue */
    uint32_t bufferHalf = BUFFER_HALF_SECOND;
    osMessageQueuePut(audioQueueHandle, &bufferHalf, 0, 0);
}

void AudioCrypto_TxHalfCpltCallback(void)
{
    /* TX callback - not used for now */
}

void AudioCrypto_TxCpltCallback(void)
{
    /* TX callback - not used for now */
}

/* ============================================================================
 * PRIVATE FUNCTIONS
 * ============================================================================ */

/**
 * @brief Prepare IV with current counter value (aligned)
 * @param counter Current CTR counter value
 */
static void PrepareIV(uint32_t counter)
{
    /* Copy base IV (first 3 words) */
    workingIV[0] = audioCtx.aesIV[0];
    workingIV[1] = audioCtx.aesIV[1];
    workingIV[2] = audioCtx.aesIV[2];

    /* Set counter in last word (big-endian for AES-CTR) */
    workingIV[3] = __REV(counter);  /* Byte-swap for big-endian */
}

/**
 * @brief Bypass mode - direct copy from input to output
 */
static HAL_StatusTypeDef ProcessBypass(uint32_t *src, uint32_t *dst, uint32_t size)
{
    /* Use word copy for better performance */
    for (uint32_t i = 0; i < size; i++) {
        dst[i] = src[i];
    }
    return HAL_OK;
}

/**
 * @brief Encrypt mode - encrypt audio data using hardware AES-CTR
 */
static HAL_StatusTypeDef ProcessEncrypt(uint32_t *src, uint32_t *dst, uint32_t size)
{
    HAL_StatusTypeDef status;

    /* Prepare aligned IV with current counter */
    PrepareIV(audioCtx.ctrCounter);

    /* Update CRYP IV (key already configured) */
    hCryp->Init.pInitVect = workingIV;

    /* DeInit and ReInit to apply new IV */
    HAL_CRYP_DeInit(hCryp);
    if (HAL_CRYP_Init(hCryp) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Encrypt data using polling (fast enough for 48kHz audio) */
    status = HAL_CRYP_Encrypt(hCryp, src, size, dst, 100);  /* 100ms timeout */

    if (status == HAL_OK) {
        /* Update counter */
        UpdateCTRCounter();
    }

    return status;
}

/**
 * @brief Encrypt-Decrypt mode - encrypt then decrypt (loopback test)
 */
static HAL_StatusTypeDef ProcessEncryptDecrypt(uint32_t *src, uint32_t *dst, uint32_t size)
{
    HAL_StatusTypeDef status;
    uint32_t savedCounter = audioCtx.ctrCounter;

    /* === ENCRYPT PHASE === */
    PrepareIV(audioCtx.ctrCounter);
    hCryp->Init.pInitVect = workingIV;

    HAL_CRYP_DeInit(hCryp);
    if (HAL_CRYP_Init(hCryp) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Encrypt to temporary buffer */
    status = HAL_CRYP_Encrypt(hCryp, src, size, audioCtx.cryptBuffer, 100);
    if (status != HAL_OK) {
        return status;
    }

    /* === DECRYPT PHASE === */
    /* Restore IV with same counter (CTR mode is symmetric) */
    PrepareIV(savedCounter);
    hCryp->Init.pInitVect = workingIV;

    HAL_CRYP_DeInit(hCryp);
    if (HAL_CRYP_Init(hCryp) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Decrypt from temporary buffer to output */
    status = HAL_CRYP_Decrypt(hCryp, audioCtx.cryptBuffer, size, dst, 100);

    if (status == HAL_OK) {
        /* Update counter */
        UpdateCTRCounter();
    }

    return status;
}

/**
 * @brief Update CTR counter for next block
 */
static void UpdateCTRCounter(void)
{
    /* Increment counter based on blocks processed
     * Each AES block is 16 bytes = 4 words
     * Half buffer size in words / 4 = number of AES blocks
     */
    uint32_t blocksProcessed = (AUDIO_DMA_BUFFER_SIZE / 2) / 4;
    audioCtx.ctrCounter += blocksProcessed;
}