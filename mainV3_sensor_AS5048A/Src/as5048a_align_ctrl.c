#include "as5048a_align_ctrl.h"
#include "speed_torq_ctrl.h"
#include "virtual_speed_sensor.h"

void AS5048A_EAC_Init(AS5048A_AlignCtrl_Handle_t *pHandle,
                      SpeednTorqCtrl_Handle_t     *pSTC,
                      VirtualSpeedSensor_Handle_t *pVSS,
                      AS5048A_SPD_Handle_t        *pSensor)
{
    if (pHandle == NULL) return;
    pHandle->pSTC    = pSTC;
    pHandle->pVSS    = pVSS;
    pHandle->pSensor = pSensor;
    pHandle->EncAligned      = false;
    pHandle->EncRestart      = false;
    pHandle->hRemainingTicks = 0u;
}

void AS5048A_EAC_StartAlignment(AS5048A_AlignCtrl_Handle_t *pHandle)
{
    if (pHandle == NULL) return;

    pHandle->EncAligned = false;
    pHandle->hRemainingTicks =
        (uint16_t)((pHandle->hDurationms * pHandle->hEACFrequencyHz) / 1000u);

    /* Force the VSS to report a fixed electrical angle.
       The rotor will be pulled toward this angle by the d-axis current. */
    pHandle->pVSS->_Super.hElAngle        = pHandle->hElAngle;
    pHandle->pVSS->_Super.hMecAngle       = pHandle->hElAngle
                                           / (int16_t)pHandle->bElToMecRatio;
    pHandle->pVSS->_Super.hAvrMecSpeedUnit = 0;
    pHandle->pVSS->_Super.hElSpeedDpp      = 0;

    /* Switch STC to torque mode and ramp up to the alignment current */
    STC_SetControlMode(pHandle->pSTC, MCM_TORQUE_MODE);
    (void)STC_ExecRamp(pHandle->pSTC, pHandle->hFinalTorque, 0u);
}

bool AS5048A_EAC_Exec(AS5048A_AlignCtrl_Handle_t *pHandle)
{
    if (pHandle == NULL) return true;

    if (pHandle->hRemainingTicks > 0u)
    {
        /* Keep holding VSS at the fixed alignment angle while rotor settles */
        pHandle->pVSS->_Super.hElAngle = pHandle->hElAngle;

        pHandle->hRemainingTicks--;

        if (pHandle->hRemainingTicks == 0u)
        {
            /* Rotor is now physically at electrical angle = hElAngle.
               Record this as the reference: compute MecAngleOffset so that
               the sensor's current reading maps to hElAngle/pp mechanically. */
            AS5048A_SPD_SetMecAngle(pHandle->pSensor,
                (int16_t)(pHandle->hElAngle
                          / (int16_t)pHandle->bElToMecRatio));
            pHandle->EncAligned = true;
            return true;
        }
        return false;
    }
    return true;
}
