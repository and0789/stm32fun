/* File: Core/Inc/audio.h */
#ifndef __AUDIO_H
#define __AUDIO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

/** @defgroup AUDIO_Exported_Types
  * @{
  */

/** @defgroup AUDIO_Driver_structure  Audio Driver structure
  * @{
  */
typedef struct
{
  uint32_t  (*Init)(uint16_t, uint16_t, uint8_t, uint32_t);
  void      (*DeInit)(void);
  uint32_t  (*ReadID)(uint16_t);
  uint32_t  (*Play)(uint16_t, uint16_t*, uint16_t);
  uint32_t  (*Pause)(uint16_t);
  uint32_t  (*Resume)(uint16_t);
  uint32_t  (*Stop)(uint16_t, uint32_t);
  uint32_t  (*SetFrequency)(uint16_t, uint32_t);
  uint32_t  (*SetVolume)(uint16_t, uint8_t);
  uint32_t  (*SetMute)(uint16_t, uint32_t);
  uint32_t  (*SetOutputMode)(uint16_t, uint8_t);
  uint32_t  (*Reset)(uint16_t);
}AUDIO_DrvTypeDef;
/**
  * @}
  */

/** @defgroup AUDIO_Exported_Constants
  * @{
  */
#define AUDIO_OK                            ((uint8_t)0)
#define AUDIO_ERROR                         ((uint8_t)1)
#define AUDIO_TIMEOUT                       ((uint8_t)2)

#define OUTPUT_DEVICE_SPEAKER               ((uint16_t)0x0001)
#define OUTPUT_DEVICE_HEADPHONE             ((uint16_t)0x0002)
#define OUTPUT_DEVICE_BOTH                  ((uint16_t)0x0003)
#define OUTPUT_DEVICE_AUTO                  ((uint16_t)0x0004)

#define AUDIO_MUTE_ON                       ((uint32_t)0x00000001)
#define AUDIO_MUTE_OFF                      ((uint32_t)0x00000000)

#define AUDIO_FREQUENCY_192K                ((uint32_t)192000)
#define AUDIO_FREQUENCY_96K                 ((uint32_t)96000)
#define AUDIO_FREQUENCY_48K                 ((uint32_t)48000)
#define AUDIO_FREQUENCY_44K                 ((uint32_t)44100)
#define AUDIO_FREQUENCY_32K                 ((uint32_t)32000)
#define AUDIO_FREQUENCY_22K                 ((uint32_t)22050)
#define AUDIO_FREQUENCY_16K                 ((uint32_t)16000)
#define AUDIO_FREQUENCY_11K                 ((uint32_t)11025)
#define AUDIO_FREQUENCY_8K                  ((uint32_t)8000)

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __AUDIO_H */
