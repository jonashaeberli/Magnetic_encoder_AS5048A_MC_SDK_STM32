/**
  ******************************************************************************
  * @file    as5048a_spd_pos_fdbk.h
  * @brief   AS5048A SPI+DMA speed & position feedback for MC SDK.
  *          Drop-in replacement for encoder_speed_pos_fdbk on STM32G4.
  ******************************************************************************
  */
#ifndef AS5048A_SPD_POS_FDBK_H
#define AS5048A_SPD_POS_FDBK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "speed_pos_fdbk.h"
#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* ── Configuration ──────────────────────────────────────────────────────── */
#define AS5048A_SPD_FIFO_DEPTH_MAX  16u

/* AS5048A SPI register addresses */
#define AS5048A_REG_NOP              0x0000u
#define AS5048A_REG_ERRFL            0x0001u
#define AS5048A_REG_PROG             0x0003u
#define AS5048A_REG_DIAG_AGC         0x3FFDu
#define AS5048A_REG_MAG              0x3FFEu
#define AS5048A_REG_ANGLE            0x3FFFu

/* SPI command bit */
#define AS5048A_CMD_READ_BIT         0x4000u
#define AS5048A_CMD_WRITE_BIT        0x0000u
#define AS5048A_PARITY_BIT           0x8000u

/* Diagnostic flags for debug */
#define AS5048A_DIAG_OK              0x0000u
#define AS5048A_DIAG_NO_COMM         0x0001u
#define AS5048A_DIAG_PARITY_ERROR    0x0002u
#define AS5048A_DIAG_ERRFL_SET       0x0004u
#define AS5048A_DIAG_MAG_LOW         0x0008u
#define AS5048A_DIAG_MAG_HIGH        0x0010u
#define AS5048A_DIAG_COF             0x0020u
#define AS5048A_DIAG_OCF_NOT_READY   0x0040u

/* ── Handle ─────────────────────────────────────────────────────────────── */
typedef struct
{
  SpeednPosFdbk_Handle_t _Super;

  SPI_HandleTypeDef *pSpi;
  GPIO_TypeDef      *CsPort;
  uint16_t           CsPin;

  DMA_HandleTypeDef  hDmaRx;
  DMA_HandleTypeDef  hDmaTx;

  uint8_t TxBuf[4];
  uint8_t RxBuf[4];

  volatile bool     DmaComplete;
  volatile bool     DataValid;
  uint16_t          RawAngle;
  int16_t           MecAngleOffset;
  int16_t           PrevMecAngle;

  uint16_t          DiagnosticFlags;
  uint16_t          RawDiagAgc;
  uint16_t          RawMag;
  uint16_t          RawErrFl;

  uint16_t SpeedSamplingFreqHz;
  uint32_t SpeedSamplingFreqUnit;
  uint8_t  SpeedBufferSize;
  int32_t  DeltaCapturesBuffer[AS5048A_SPD_FIFO_DEPTH_MAX];
  uint8_t  DeltaCapturesIndex;
  int16_t  PrevSpeedMecAngle;
  bool     SensorIsReliable;

  bool     Initialized;

} AS5048A_SPD_Handle_t;

/* ── API ────────────────────────────────────────────────────────────────── */
void AS5048A_SPD_Init(AS5048A_SPD_Handle_t *pHandle);
void AS5048A_SPD_Clear(AS5048A_SPD_Handle_t *pHandle);
int16_t AS5048A_SPD_CalcAngle(AS5048A_SPD_Handle_t *pHandle);
bool AS5048A_SPD_CalcAvrgMecSpeedUnit(AS5048A_SPD_Handle_t *pHandle,
                                      int16_t *pMecSpeedUnit);
void AS5048A_SPD_SetMecAngle(AS5048A_SPD_Handle_t *pHandle, int16_t hMecAngle);
void AS5048A_SPD_DmaCompleteCallback(AS5048A_SPD_Handle_t *pHandle);

/* Debug helpers */
uint16_t AS5048A_SPD_GetDiagnosticFlags(AS5048A_SPD_Handle_t *pHandle);
uint16_t AS5048A_SPD_GetRawAngle(AS5048A_SPD_Handle_t *pHandle);

#ifdef __cplusplus
}
#endif
#endif /* AS5048A_SPD_POS_FDBK_H */