/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_audio_if.c
  * @version        : v1.0_Cube
  * @brief          : Generic media access layer.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_audio_if.h"

/* USER CODE BEGIN INCLUDE */
#include "cs43l22.h"
#include "main.h"
/* USER CODE END INCLUDE */

/* USER CODE BEGIN PRIVATE_TYPES */
extern I2S_HandleTypeDef hi2s3;
extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PRIVATE_TYPES */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */

/* USER CODE BEGIN PRIVATE_VARIABLES */
static uint8_t isPlaying = 0;
static uint8_t isCodecInitialized = 0;
/* USER CODE END PRIVATE_VARIABLES */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* Private function prototypes */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options);
static int8_t AUDIO_DeInit_FS(uint32_t options);
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol);
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd);
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd);
static int8_t AUDIO_GetState_FS(void);

USBD_AUDIO_ItfTypeDef USBD_AUDIO_fops_FS =
{
  AUDIO_Init_FS,
  AUDIO_DeInit_FS,
  AUDIO_AudioCmd_FS,
  AUDIO_VolumeCtl_FS,
  AUDIO_MuteCtl_FS,
  AUDIO_PeriodicTC_FS,
  AUDIO_GetState_FS,
};

/**
  * @brief  Initializes the AUDIO media low layer over USB FS IP
  * @param  AudioFreq: Audio frequency used to play the audio stream.
  * @param  Volume: Initial volume level (from 0 (Mute) to 100 (Max))
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_Init_FS(uint32_t AudioFreq, uint32_t Volume, uint32_t options)
{
  /* USER CODE BEGIN 0 */
  UNUSED(AudioFreq);
  UNUSED(Volume);
  UNUSED(options);

  // JANGAN init hardware di sini - hanya return OK
  // Hardware init dilakukan di main.c atau saat PLAY command

  return (USBD_OK);
  /* USER CODE END 0 */
}

/**
  * @brief  De-Initializes the AUDIO media low layer
  * @param  options: Reserved for future use
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_DeInit_FS(uint32_t options)
{
  /* USER CODE BEGIN 1 */
  UNUSED(options);

  // Stop I2S DMA jika sedang berjalan
  if (isPlaying) {
    HAL_I2S_DMAStop(&hi2s3);
    isPlaying = 0;
  }

  // Power down codec jika sudah diinit
  if (isCodecInitialized) {
    cs43l22_Stop(CS43L22_I2C_ADDRESS, CS43L22_PDWN_SW);
  }

  return (USBD_OK);
  /* USER CODE END 1 */
}

/**
  * @brief  Handles AUDIO command.
  * @param  pbuf: Pointer to buffer of data to be sent
  * @param  size: Number of data to be sent (in bytes)
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_AudioCmd_FS(uint8_t* pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 2 */
  switch(cmd)
  {
    case AUDIO_CMD_START:
    case AUDIO_CMD_PLAY:
      // Init codec saat pertama kali play (lazy initialization)
      if (!isCodecInitialized) {
        // Pastikan Reset pin HIGH
        HAL_GPIO_WritePin(GPIOD, Audio_RST_Pin, GPIO_PIN_SET);
        HAL_Delay(5);

        // Init codec
        cs43l22_Init(CS43L22_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, 80, AUDIO_FREQUENCY_48K);
        isCodecInitialized = 1;
      }

      if (!isPlaying) {
        // Unmute dan power up codec
        cs43l22_Play(CS43L22_I2C_ADDRESS, NULL, 0);
        isPlaying = 1;
      }

      // Kirim data ke I2S jika ada
      if (pbuf != NULL && size > 0) {
        // size dibagi 2: byte -> 16-bit words
        HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)pbuf, size / 2);
      }
      break;

    case AUDIO_CMD_STOP:
      if (isPlaying) {
        // Hentikan DMA
        HAL_I2S_DMAStop(&hi2s3);

        // Mute codec (tapi jangan power down untuk menghindari pop noise)
        cs43l22_SetMute(CS43L22_I2C_ADDRESS, AUDIO_MUTE_ON);

        isPlaying = 0;
      }
      break;

    default:
      break;
  }

  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Controls AUDIO Volume.
  * @param  vol: volume level (0..100)
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_VolumeCtl_FS(uint8_t vol)
{
  /* USER CODE BEGIN 3 */
  if (isCodecInitialized) {
    cs43l22_SetVolume(CS43L22_I2C_ADDRESS, vol);
  }
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Controls AUDIO Mute.
  * @param  cmd: command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_MuteCtl_FS(uint8_t cmd)
{
  /* USER CODE BEGIN 4 */
  if (isCodecInitialized) {
    if (cmd == AUDIO_MUTE_ON) {
      cs43l22_SetMute(CS43L22_I2C_ADDRESS, AUDIO_MUTE_ON);
    } else {
      cs43l22_SetMute(CS43L22_I2C_ADDRESS, AUDIO_MUTE_OFF);
    }
  }
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  AUDIO_PeriodicTC_FS
  * @param  pbuf: Pointer to audio data buffer
  * @param  size: Size of data
  * @param  cmd: Command opcode
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_PeriodicTC_FS(uint8_t *pbuf, uint32_t size, uint8_t cmd)
{
  /* USER CODE BEGIN 5 */
  UNUSED(cmd);

  // Fungsi ini dipanggil secara periodik dengan data audio dari host
  if (isPlaying && pbuf != NULL && size > 0) {
    // Kirim data ke I2S via DMA
    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t*)pbuf, size / 2);
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Gets AUDIO State.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t AUDIO_GetState_FS(void)
{
  /* USER CODE BEGIN 6 */
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  Manages the DMA full transfer complete event.
  * @retval None
  */
void TransferComplete_CallBack_FS(void)
{
  /* USER CODE BEGIN 7 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_FULL);
  /* USER CODE END 7 */
}

/**
  * @brief  Manages the DMA Half transfer complete event.
  * @retval None
  */
void HalfTransfer_CallBack_FS(void)
{
  /* USER CODE BEGIN 8 */
  USBD_AUDIO_Sync(&hUsbDeviceFS, AUDIO_OFFSET_HALF);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @brief  Callback saat Transmisi I2S Selesai (Full Transfer)
  */
// void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s) {
//   if(hi2s->Instance == SPI3) {
//     TransferComplete_CallBack_FS();
//   }
// }
//
// /**
//   * @brief  Callback saat Transmisi I2S Setengah Jalan (Half Transfer)
//   */
// void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
//   if(hi2s->Instance == SPI3) {
//     HalfTransfer_CallBack_FS();
//   }
// }

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */