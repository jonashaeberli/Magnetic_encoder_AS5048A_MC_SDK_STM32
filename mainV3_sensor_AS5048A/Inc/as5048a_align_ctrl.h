/**
  ******************************************************************************
  * @file    as5048a_align_ctrl.h
  * @brief   Encoder alignment controller adapted for AS5048A absolute sensor.
  *          Mirror of enc_align_ctrl.h with AS5048A_SPD_Handle_t in place of
  *          ENCODER_Handle_t.
  ******************************************************************************
  */
#ifndef AS5048A_ALIGN_CTRL_H
#define AS5048A_ALIGN_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "as5048a_spd_pos_fdbk.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"
#include <stdbool.h>

typedef struct
{
  SpeednTorqCtrl_Handle_t      *pSTC;
  VirtualSpeedSensor_Handle_t  *pVSS;
  AS5048A_SPD_Handle_t         *pSensor;   /**< AS5048A sensor handle */

  bool     EncAligned;
  bool     EncRestart;

  int16_t  hFinalTorque;     /**< Alignment torque (digit)                  */
  int16_t  hElAngle;         /**< Target electrical angle during alignment  */
  uint16_t hDurationms;      /**< Alignment duration [ms]                   */
  uint16_t hEACFrequencyHz;  /**< = MEDIUM_FREQUENCY_TASK_RATE              */
  uint8_t  bElToMecRatio;    /**< = POLE_PAIR_NUM                           */
  uint16_t hRemainingTicks;  /**< Counts down during alignment              */
} AS5048A_AlignCtrl_Handle_t;

void AS5048A_EAC_Init(AS5048A_AlignCtrl_Handle_t *pHandle,
                      SpeednTorqCtrl_Handle_t *pSTC,
                      VirtualSpeedSensor_Handle_t *pVSS,
                      AS5048A_SPD_Handle_t *pSensor);

void AS5048A_EAC_StartAlignment(AS5048A_AlignCtrl_Handle_t *pHandle);
bool AS5048A_EAC_Exec(AS5048A_AlignCtrl_Handle_t *pHandle);

static inline bool AS5048A_EAC_IsAligned(const AS5048A_AlignCtrl_Handle_t *p)
{
  return (p != NULL) ? p->EncAligned : false;
}

static inline bool AS5048A_EAC_GetRestartState(const AS5048A_AlignCtrl_Handle_t *p)
{
  return (p != NULL) ? p->EncRestart : false;
}

#ifdef __cplusplus
}
#endif
#endif /* AS5048A_ALIGN_CTRL_H */