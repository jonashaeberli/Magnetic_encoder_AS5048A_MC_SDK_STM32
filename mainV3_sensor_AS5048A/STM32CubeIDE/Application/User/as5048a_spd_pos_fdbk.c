/**
  ******************************************************************************
  * @file    as5048a_spd_pos_fdbk.c
  * @brief   AS5048A SPI+DMA speed & position feedback – implementation.
  ******************************************************************************
  */

/* Includes ----------------------------------------------------------------- */
#include "as5048a_spd_pos_fdbk.h"
#include "mc_type.h"
#include <string.h>

#define SPEED_LPF_ALPHA   200   /* out of 256: higher = less filtering */

/* ── Private helpers ------------------------------------------------------- */

static uint8_t as5048a_odd_parity16(uint16_t x)
{
  x ^= x >> 8;
  x ^= x >> 4;
  x &= 0x0Fu;
  return (0x6996u >> x) & 1u;
}

static uint16_t as5048a_build_cmd(uint16_t addr, bool read)
{
  uint16_t cmd = (addr & 0x3FFFu);
  if (read)
  {
    cmd |= AS5048A_CMD_READ_BIT;
  }

  if (as5048a_odd_parity16(cmd))
  {
    cmd |= AS5048A_PARITY_BIT;
  }

  return cmd;
}

static inline void cs_assert(AS5048A_SPD_Handle_t *p)
{
  HAL_GPIO_WritePin(p->CsPort, p->CsPin, GPIO_PIN_RESET);
}

static inline void cs_deassert(AS5048A_SPD_Handle_t *p)
{
  HAL_GPIO_WritePin(p->CsPort, p->CsPin, GPIO_PIN_SET);
}

static uint16_t as5048a_blocking_transfer(AS5048A_SPD_Handle_t *p, uint16_t cmd_word)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = (uint8_t)(cmd_word >> 8);
  tx[1] = (uint8_t)(cmd_word & 0xFFu);

  cs_assert(p);
  HAL_SPI_TransmitReceive(p->pSpi, tx, rx, 2, 10);
  cs_deassert(p);

  return ((uint16_t)rx[0] << 8) | rx[1];
}

static uint16_t as5048a_read_reg_blocking(AS5048A_SPD_Handle_t *p, uint16_t addr)
{
  (void)as5048a_blocking_transfer(p, as5048a_build_cmd(addr, true));
  return as5048a_blocking_transfer(p, as5048a_build_cmd(AS5048A_REG_NOP, false));
}

static void as5048a_dma_init(AS5048A_SPD_Handle_t *pHandle)
{
    __HAL_RCC_DMA2_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    /* DMA2 Channel 1 — SPI3_RX */
    pHandle->hDmaRx.Instance                 = DMA2_Channel1;
    pHandle->hDmaRx.Init.Request             = DMA_REQUEST_SPI3_RX;
    pHandle->hDmaRx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    pHandle->hDmaRx.Init.PeriphInc           = DMA_PINC_DISABLE;
    pHandle->hDmaRx.Init.MemInc              = DMA_MINC_ENABLE;
    pHandle->hDmaRx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    pHandle->hDmaRx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    pHandle->hDmaRx.Init.Mode                = DMA_NORMAL;
    pHandle->hDmaRx.Init.Priority            = DMA_PRIORITY_HIGH;
    //pHandle->hDmaRx.XferHalfCpltCallback	 = HAL_DMA_HALF_TRANSFER;
    if (HAL_DMA_Init(&pHandle->hDmaRx) != HAL_OK) {
    	//Error_Handler();
    }
    /* DMA2 Channel 2 — SPI3_TX */
    pHandle->hDmaTx.Instance                 = DMA2_Channel2;
    pHandle->hDmaTx.Init.Request             = DMA_REQUEST_SPI3_TX;
    pHandle->hDmaTx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
    pHandle->hDmaTx.Init.PeriphInc           = DMA_PINC_DISABLE;
    pHandle->hDmaTx.Init.MemInc              = DMA_MINC_ENABLE;
    pHandle->hDmaTx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    pHandle->hDmaTx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    pHandle->hDmaTx.Init.Mode                = DMA_NORMAL;
    pHandle->hDmaTx.Init.Priority            = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&pHandle->hDmaTx) != HAL_OK) {
    	//Error_Handler();
    }
    /* Link to SPI handle */
    __HAL_LINKDMA(pHandle->pSpi, hdmarx, pHandle->hDmaRx);
    __HAL_LINKDMA(pHandle->pSpi, hdmatx, pHandle->hDmaTx);

    /* NVIC: priority (2,1) — below FOC ADC ISR */
    HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);
    HAL_NVIC_SetPriority(DMA2_Channel2_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(DMA2_Channel2_IRQn);
}

static void as5048a_start_dma_read(AS5048A_SPD_Handle_t *p)
{
  if (p->pSpi->State != HAL_SPI_STATE_READY)
  {
    return;
  }

  p->DmaComplete = false;
  cs_assert(p);
  HAL_SPI_TransmitReceive_DMA(p->pSpi, p->TxBuf, p->RxBuf, 2);
}

static void as5048a_run_diagnostics(AS5048A_SPD_Handle_t *p)
{
  p->DiagnosticFlags = AS5048A_DIAG_OK;

  p->RawErrFl  = as5048a_read_reg_blocking(p, AS5048A_REG_ERRFL);
  p->RawDiagAgc = as5048a_read_reg_blocking(p, AS5048A_REG_DIAG_AGC);
  p->RawMag    = as5048a_read_reg_blocking(p, AS5048A_REG_MAG);
  p->RawAngle  = as5048a_read_reg_blocking(p, AS5048A_REG_ANGLE) & 0x3FFFu;

  /* ERRFL bits: parity, invalid command, framing error */
  if (p->RawErrFl & 0x0007u)
  {
    p->DiagnosticFlags |= AS5048A_DIAG_ERRFL_SET;
  }

  /* Diagnostics register interpretation */
  if (p->RawDiagAgc & (1u << 10))
  {
    p->DiagnosticFlags |= AS5048A_DIAG_MAG_HIGH;
  }
  if (p->RawDiagAgc & (1u << 11))
  {
    p->DiagnosticFlags |= AS5048A_DIAG_MAG_LOW;
  }
  if (p->RawDiagAgc & (1u << 9))
  {
    p->DiagnosticFlags |= AS5048A_DIAG_COF;
  }
  if ((p->RawDiagAgc & (1u << 8)) == 0u)
  {
    p->DiagnosticFlags |= AS5048A_DIAG_OCF_NOT_READY;
  }
}

/* ── Public API ----------------------------------------------------------- */

void AS5048A_SPD_Init(AS5048A_SPD_Handle_t *pHandle)
{
  if (pHandle == NULL)
  {
    return;
  }

  uint16_t angle_cmd = as5048a_build_cmd(AS5048A_REG_ANGLE, true); /* = 0xFFFF */
  pHandle->TxBuf[0] = (uint8_t)(angle_cmd >> 8);
  pHandle->TxBuf[1] = (uint8_t)(angle_cmd & 0xFFu);

  cs_deassert(pHandle);
  HAL_Delay(1);

  /* Init-time diagnostics */
  as5048a_run_diagnostics(pHandle);

  as5048a_dma_init(pHandle);



  pHandle->DataValid = true;
  pHandle->MecAngleOffset = 0;

  int16_t mecAngle = (int16_t)(pHandle->RawAngle * 4u); /* 14-bit -> s16degree */
  pHandle->_Super.hMecAngle = mecAngle;
  pHandle->_Super.hElAngle  = mecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
  pHandle->_Super.wMecAngle = 0;
  pHandle->PrevMecAngle = mecAngle;
  pHandle->PrevSpeedMecAngle = mecAngle;

  pHandle->SpeedSamplingFreqUnit =
      (uint32_t)pHandle->SpeedSamplingFreqHz * (uint32_t)SPEED_UNIT;
  pHandle->DeltaCapturesIndex = 0;
  pHandle->SensorIsReliable = (pHandle->DiagnosticFlags == AS5048A_DIAG_OK);

  /* Prime the pipeline: one blocking READ_ANGLE so the next DMA frame
         receives valid angle data immediately rather than a stale NOP response */
  as5048a_blocking_transfer(pHandle, as5048a_build_cmd(AS5048A_REG_ANGLE, true)); /* discard — primes pipeline */


  /* Start first DMA pipeline transfer */
  pHandle->DmaComplete = false;
  as5048a_start_dma_read(pHandle);

  pHandle->Initialized = true;
}

void AS5048A_SPD_Clear(AS5048A_SPD_Handle_t *pHandle)
{
  if (pHandle == NULL)
  {
    return;
  }

  for (uint8_t i = 0; i < pHandle->SpeedBufferSize; i++)
  {
    pHandle->DeltaCapturesBuffer[i] = 0;
  }

  pHandle->DeltaCapturesIndex = 0;
  pHandle->SensorIsReliable   = true;
  pHandle->_Super.bSpeedErrorNumber = 0;
  pHandle->_Super.hAvrMecSpeedUnit  = 0;
  pHandle->_Super.hElSpeedDpp       = 0;
  pHandle->_Super.hMecAccelUnitP    = 0;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
int16_t AS5048A_SPD_CalcAngle(AS5048A_SPD_Handle_t *pHandle)
{
    int16_t elAngle;

    if (pHandle->DmaComplete)
    {
        uint16_t rawWord = ((uint16_t)pHandle->RxBuf[0] << 8) | pHandle->RxBuf[1];
        pHandle->RawAngle = (rawWord & 0x3FFFu);
        pHandle->DataValid = true;
    }

    /* Use RawAngle directly — no inversion */
    int16_t mecAngle = (int16_t)(pHandle->RawAngle << 2) + pHandle->MecAngleOffset;

    int16_t hMecAnglePrev = pHandle->_Super.hMecAngle;
    pHandle->_Super.hMecAngle = mecAngle;

    elAngle = mecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
    pHandle->_Super.hElAngle = elAngle;

    /* Wraparound-safe accumulator (keep this — needed regardless) */
    int32_t hMecSpeedDpp = (int32_t)mecAngle - (int32_t)hMecAnglePrev;
    if (hMecSpeedDpp > 32767)        hMecSpeedDpp -= 65536;
    else if (hMecSpeedDpp < -32768)  hMecSpeedDpp += 65536;
    pHandle->_Super.wMecAngle += hMecSpeedDpp;

    as5048a_start_dma_read(pHandle);
    return elAngle;
}

bool AS5048A_SPD_CalcAvrgMecSpeedUnit(AS5048A_SPD_Handle_t *pHandle,
                                      int16_t *pMecSpeedUnit)
{
    bool bReliability;

    int16_t currentMecAngle = pHandle->_Super.hMecAngle;
    int32_t delta = (int32_t)currentMecAngle - (int32_t)pHandle->PrevSpeedMecAngle;

    /* ── Wraparound correction ──
       hMecAngle wraps every mechanical revolution at ±32768.
       Any single-sample delta exceeding half a revolution is a wraparound,
       not a real motion.  Fold it back into the [-32768, +32767] range. */
    if (delta > 32767)
    {
        delta -= 65536;
    }
    else if (delta < -32768)
    {
        delta += 65536;
    }

    pHandle->DeltaCapturesBuffer[pHandle->DeltaCapturesIndex] = delta;
    pHandle->PrevSpeedMecAngle = currentMecAngle;

    int32_t wOverallAngleVariation = 0;
    for (uint8_t i = 0; i < pHandle->SpeedBufferSize; i++)
    {
        wOverallAngleVariation += pHandle->DeltaCapturesBuffer[i];
    }

    int32_t wtemp1 = wOverallAngleVariation
                   * (int32_t)pHandle->SpeedSamplingFreqUnit;
    int32_t wtemp2 = (int32_t)65536 * (int32_t)pHandle->SpeedBufferSize;
    wtemp1 = (wtemp2 == 0) ? 0 : (wtemp1 / wtemp2);

    *pMecSpeedUnit = (int16_t)wtemp1;

    pHandle->_Super.hMecAccelUnitP =
        (int16_t)(wtemp1 - (int32_t)pHandle->_Super.hAvrMecSpeedUnit);
    pHandle->_Super.hAvrMecSpeedUnit = (int16_t)wtemp1;

    int32_t dpp = delta
                * (int32_t)pHandle->SpeedSamplingFreqHz
                * (int32_t)pHandle->_Super.bElToMecRatio;
    dpp /= (int32_t)65536;
    dpp *= (int32_t)pHandle->_Super.DPPConvFactor;
    dpp /= (int32_t)pHandle->_Super.hMeasurementFrequency;
    pHandle->_Super.hElSpeedDpp = (int16_t)dpp;

    pHandle->DeltaCapturesIndex++;
    if (pHandle->DeltaCapturesIndex >= pHandle->SpeedBufferSize)
    {
        pHandle->DeltaCapturesIndex = 0;
    }

    bReliability = SPD_IsMecSpeedReliable(&pHandle->_Super, pMecSpeedUnit);
    pHandle->SensorIsReliable = bReliability && (pHandle->DiagnosticFlags == AS5048A_DIAG_OK);

    return pHandle->SensorIsReliable;
}

void AS5048A_SPD_SetMecAngle(AS5048A_SPD_Handle_t *pHandle, int16_t hMecAngle)
{
  if (pHandle == NULL)
  {
    return;
  }

  pHandle->MecAngleOffset = hMecAngle - (int16_t)(pHandle->RawAngle << 2);

  pHandle->_Super.hMecAngle = hMecAngle;
  pHandle->_Super.hElAngle  = hMecAngle * (int16_t)pHandle->_Super.bElToMecRatio;
  pHandle->_Super.wMecAngle = 0;
  pHandle->PrevMecAngle     = hMecAngle;
  pHandle->PrevSpeedMecAngle = hMecAngle;
}

void AS5048A_SPD_DmaCompleteCallback(AS5048A_SPD_Handle_t *pHandle)
{
  cs_deassert(pHandle);
  pHandle->DmaComplete = true;
}

/* ── Debug helpers ──────────────────────────────────────────────────────── */

uint16_t AS5048A_SPD_GetDiagnosticFlags(AS5048A_SPD_Handle_t *pHandle)
{
  if (pHandle == NULL)
  {
    return AS5048A_DIAG_NO_COMM;
  }
  return pHandle->DiagnosticFlags;
}

uint16_t AS5048A_SPD_GetRawAngle(AS5048A_SPD_Handle_t *pHandle)
{
  if (pHandle == NULL)
  {
    return 0;
  }
  return pHandle->RawAngle;
}
