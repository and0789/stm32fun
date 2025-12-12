/**
 * @file audio_crypto.c
 * @brief Audio Encryption/Decryption Implementation using STM32H750 Hardware CRYP
 * @author Jerry
 * @date 2024
 * @version 2.0 - Fixed alignment issues and optimized for interrupt context
 */

#include "audio_crypto.h"
#include <string.h>
#include <stdio.h>

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
static uint32_t workingIV[4] __attribute__((section(".RAM_D2"), aligned(4)));

/** Mode names for debug output */
static const char* MODE_NAMES[] = {
    "BYPASS",
    "ENCRYPT",
    "ENCRYPT+DECRYPT"
};

/** Flag to indicate processing is needed (for deferred processing) */
static volatile uint8_t pendingProcess = 0;
static volatile BufferHalf_t pendingBufferHalf = BUFFER_HALF_FIRST;

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
    
    /* Set default key and IV (already in uint32_t format) */
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
    
    /* Reset flags */
    audioCtx.rxHalfComplete = false;
    audioCtx.rxFullComplete = false;
    audioCtx.txHalfComplete = false;
    audioCtx.txFullComplete = false;
    pendingProcess = 0;
    
    /* Pre-configure CRYP with default key (avoid init in interrupt) */
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
        audioCtx.mode = mode;
        audioCtx.ctrCounter = 0;  /* Reset counter on mode change */
        printf("[AudioCrypto] Mode changed to: %s\r\n", MODE_NAMES[mode]);
    }
}

AudioMode_t AudioCrypto_GetMode(void)
{
    return audioCtx.mode;
}

void AudioCrypto_NextMode(void)
{
    AudioMode_t newMode = (audioCtx.mode + 1) % AUDIO_MODE_COUNT;
    AudioCrypto_SetMode(newMode);
}

const char* AudioCrypto_GetModeName(AudioMode_t mode)
{
    if (mode < AUDIO_MODE_COUNT) {
        return MODE_NAMES[mode];
    }
    return "UNKNOWN";
}

void AudioCrypto_SetKey(const uint32_t *key)
{
    if (key != NULL) {
        memcpy(audioCtx.aesKey, key, sizeof(audioCtx.aesKey));
        audioCtx.ctrCounter = 0;  /* Reset counter on key change */
        
        /* Update CRYP configuration */
        hCryp->Init.pKey = audioCtx.aesKey;
        HAL_CRYP_Init(hCryp);
        
        printf("[AudioCrypto] AES key updated\r\n");
    }
}

void AudioCrypto_SetIV(const uint32_t *iv)
{
    if (iv != NULL) {
        memcpy(audioCtx.aesIV, iv, sizeof(audioCtx.aesIV));
        audioCtx.ctrCounter = 0;  /* Reset counter on IV change */
        printf("[AudioCrypto] AES IV updated\r\n");
    }
}

void AudioCrypto_ProcessBuffer(BufferHalf_t bufferHalf)
{
    uint32_t *srcPtr;
    uint32_t *dstPtr;
    uint32_t halfSize = AUDIO_DMA_BUFFER_SIZE / 2;
    HAL_StatusTypeDef status = HAL_OK;
    
    /* Determine buffer pointers based on which half to process */
    if (bufferHalf == BUFFER_HALF_FIRST) {
        srcPtr = &audioCtx.rxBuffer[0];
        dstPtr = &audioCtx.txBuffer[0];
    } else {
        srcPtr = &audioCtx.rxBuffer[halfSize];
        dstPtr = &audioCtx.txBuffer[halfSize];
    }
    
    /* Process based on current mode */
    switch (audioCtx.mode) {
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
 * CALLBACK FUNCTIONS (called from HAL callbacks)
 * ============================================================================ */

void AudioCrypto_RxHalfCpltCallback(void)
{
    audioCtx.rxHalfComplete = true;
    AudioCrypto_ProcessBuffer(BUFFER_HALF_FIRST);
}

void AudioCrypto_RxCpltCallback(void)
{
    audioCtx.rxFullComplete = true;
    AudioCrypto_ProcessBuffer(BUFFER_HALF_SECOND);
}

void AudioCrypto_TxHalfCpltCallback(void)
{
    audioCtx.txHalfComplete = true;
}

void AudioCrypto_TxCpltCallback(void)
{
    audioCtx.txFullComplete = true;
}

void AudioCrypto_ButtonCallback(void)
{
    AudioCrypto_NextMode();
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
 * @brief Encrypt-Decrypt mode - encrypt then decrypt (loopback test) for production
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

// static HAL_StatusTypeDef ProcessEncryptDecrypt(uint32_t *src, uint32_t *dst, uint32_t size)
// {
//     HAL_StatusTypeDef status;
//     uint32_t savedCounter = audioCtx.ctrCounter;
//
//     /* ============================ */
//     /* 1. ENCRYPT PHASE        */
//     /* ============================ */
//
//     /* Matikan peripheral dulu untuk reset state */
//     HAL_CRYP_DeInit(hCryp);
//
//     /* Siapkan IV untuk Enkripsi */
//     PrepareIV(audioCtx.ctrCounter);
//
//     /* Konfigurasi ulang parameter */
//     hCryp->Init.pInitVect = workingIV;
//     hCryp->Init.DataType = CRYP_DATATYPE_32B; /* Pastikan mode 32-bit (No Swap) */
//     hCryp->Init.KeySize = CRYP_KEYSIZE_128B;
//     hCryp->Init.Algorithm = CRYP_AES_CTR;
//     hCryp->Init.pKey = audioCtx.aesKey;       /* Pastikan Key ter-load ulang */
//
//     if (HAL_CRYP_Init(hCryp) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     /* MANUAL FORCE IV WRITE:
//      * Memastikan register hardware benar-benar terisi nilai Counter baru.
//      * Dilakukan setelah Init berhasil.
//      */
//     hCryp->Instance->IV0LR = workingIV[0];
//     hCryp->Instance->IV0RR = workingIV[1];
//     hCryp->Instance->IV1LR = workingIV[2];
//     hCryp->Instance->IV1RR = workingIV[3];
//
//     /* Lakukan Enkripsi (Src -> CryptBuffer) */
//     status = HAL_CRYP_Encrypt(hCryp, src, size, audioCtx.cryptBuffer, 100);
//     if (status != HAL_OK) return status;
//
//
//     /* ============================ */
//     /* 2. DECRYPT PHASE        */
//     /* ============================ */
//
//     /* Reset lagi untuk fase dekripsi */
//     HAL_CRYP_DeInit(hCryp);
//
//     /* KEMBALIKAN Counter ke nilai lama (Mundur ke masa lalu) */
//     PrepareIV(savedCounter);
//
//     hCryp->Init.pInitVect = workingIV;
//     hCryp->Init.DataType = CRYP_DATATYPE_32B;
//
//     if (HAL_CRYP_Init(hCryp) != HAL_OK) {
//         return HAL_ERROR;
//     }
//
//     /* MANUAL FORCE IV WRITE (Lagi untuk Dekripsi) */
//     hCryp->Instance->IV0LR = workingIV[0];
//     hCryp->Instance->IV0RR = workingIV[1];
//     hCryp->Instance->IV1LR = workingIV[2];
//     hCryp->Instance->IV1RR = workingIV[3];
//
//     /* Lakukan Dekripsi (CryptBuffer -> Dst) */
//     status = HAL_CRYP_Decrypt(hCryp, audioCtx.cryptBuffer, size, dst, 100);
//
//     /* ============================ */
//     /* 3. DEBUG SNAPSHOT       */
//     /* ============================ */
//     if (status == HAL_OK) {
//         UpdateCTRCounter();
//
//         static uint32_t lastPrintTick = 0;
//         /* Print tiap 2 detik */
//         if (HAL_GetTick() - lastPrintTick > 2000) {
//             lastPrintTick = HAL_GetTick();
//
//             printf("\r\n--- AUDIO DATA SNAPSHOT (Hex) ---\r\n");
//             printf("Idx | ORIGINAL   | ENCRYPTED  | DECRYPTED\r\n");
//             printf("----|------------|------------|------------\r\n");
//             for (int i = 0; i < 4; i++) {
//                 printf("%03d | 0x%08lX | 0x%08lX | 0x%08lX\r\n",
//                        i, src[i], audioCtx.cryptBuffer[i], dst[i]);
//             }
//             printf("-----------------------------------\r\n");
//
//             /* Cek Match */
//             if (src[0] == dst[0]) {
//                 printf("[OK] Data MATCH!\r\n");
//             } else {
//                 printf("[FAIL] Data MISMATCH!\r\n");
//             }
//         }
//     }
//
//     return status;
// }

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
