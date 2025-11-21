/* File: Core/Inc/cs43l22.h */

#ifndef __CS43L22_H
#define __CS43L22_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "audio.h"
#include "stm32f4xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup CS43L22
  * @{
  */

/** @defgroup CS43L22_Exported_Constants
  * @{
  */

/* Definisi Alamat I2C dan Standar Audio */
#define CS43L22_I2C_ADDRESS           0x94
#define CODEC_STANDARD                0x04

#define CS43L22_PDWN_HW               0x01
#define CS43L22_PDWN_SW               0x02

/* CS43L22 Chip ID */
#define CS43L22_ID            0xE0
#define CS43L22_ID_MASK       0xF8

/* CS43L22 Registers */
#define CS43L22_CHIPID_ADDR    0x01
#define CS43L22_REG_POWER_CTL1 0x02
#define CS43L22_REG_POWER_CTL2 0x04
#define CS43L22_REG_CLOCKING_CTL 0x05
#define CS43L22_REG_INTERFACE_CTL1 0x06
#define CS43L22_REG_INTERFACE_CTL2 0x07
#define CS43L22_REG_PASSTHR_A_SEL 0x08
#define CS43L22_REG_PASSTHR_B_SEL 0x09
#define CS43L22_REG_ANALOG_ZC_SR_SETT 0x0A
#define CS43L22_REG_PASSTHR_GANG_CTL 0x0C
#define CS43L22_REG_PLAYBACK_CTL1 0x0D
#define CS43L22_REG_MISC_CTL 0x0E
#define CS43L22_REG_PLAYBACK_CTL2 0x0F
#define CS43L22_REG_PASSTHR_A_VOL 0x14
#define CS43L22_REG_PASSTHR_B_VOL 0x15
#define CS43L22_REG_PCMA_VOL 0x1A
#define CS43L22_REG_PCMB_VOL 0x1B
#define CS43L22_REG_BEEP_FREQ_ON_TIME 0x1C
#define CS43L22_REG_BEEP_VOL_OFF_TIME 0x1D
#define CS43L22_REG_BEEP_TONE_CFG 0x1E
#define CS43L22_REG_TONE_CTL 0x1F
#define CS43L22_REG_MASTER_A_VOL 0x20
#define CS43L22_REG_MASTER_B_VOL 0x21
#define CS43L22_REG_HEADPHONE_A_VOL 0x22
#define CS43L22_REG_HEADPHONE_B_VOL 0x23
#define CS43L22_REG_SPEAKER_A_VOL 0x24
#define CS43L22_REG_SPEAKER_B_VOL 0x25
#define CS43L22_REG_CH_MIX_SWAP 0x26
#define CS43L22_REG_LIMIT_CTL1 0x27
#define CS43L22_REG_LIMIT_CTL2 0x28
#define CS43L22_REG_LIMIT_ATTACK_RATE 0x29
#define CS43L22_REG_OVF_CLK_STATUS 0x2E
#define CS43L22_REG_BATT_COMP 0x2F
#define CS43L22_REG_VP_BATT_LEVEL 0x30
#define CS43L22_REG_SPEAKER_STATUS 0x31
#define CS43L22_REG_TEMPMONITOR_CTL 0x32
#define CS43L22_REG_THERMAL_FOLDBACK 0x33
#define CS43L22_REG_CHARGE_PUMP_FREQ 0x34

/**
  * @}
  */

/** @defgroup CS43L22_Exported_Functions
  * @{
  */

uint32_t cs43l22_Init(uint16_t DeviceAddr, uint16_t OutputDevice, uint8_t Volume,  uint32_t AudioFreq);
void     cs43l22_DeInit(void);
uint32_t cs43l22_ReadID(uint16_t DeviceAddr);
uint32_t cs43l22_Play(uint16_t DeviceAddr, uint16_t* pBuffer, uint16_t Size);
uint32_t cs43l22_Pause(uint16_t DeviceAddr);
uint32_t cs43l22_Resume(uint16_t DeviceAddr);
uint32_t cs43l22_Stop(uint16_t DeviceAddr, uint32_t CodecPdwnMode);
uint32_t cs43l22_SetVolume(uint16_t DeviceAddr, uint8_t Volume);
uint32_t cs43l22_SetFrequency(uint16_t DeviceAddr, uint32_t AudioFreq);
uint32_t cs43l22_SetMute(uint16_t DeviceAddr, uint32_t Cmd);
uint32_t cs43l22_SetOutputMode(uint16_t DeviceAddr, uint8_t Output);
uint32_t cs43l22_Reset(uint16_t DeviceAddr);

/* Audio IO functions */
void    AUDIO_IO_Init(void);
void    AUDIO_IO_DeInit(void);
void    AUDIO_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value);
uint8_t AUDIO_IO_Read(uint8_t Addr, uint8_t Reg);

#ifdef __cplusplus
}
#endif

#endif /* __CS43L22_H */