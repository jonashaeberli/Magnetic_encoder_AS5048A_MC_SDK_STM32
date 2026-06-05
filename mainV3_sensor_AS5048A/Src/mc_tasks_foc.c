/**
  ******************************************************************************
  * @file    mc_tasks_foc.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements tasks definition.
  *          *** MODIFIED: AS5048A SPI+DMA replaces quadrature encoder ***
  ******************************************************************************
  */

/* Includes ----------------------------------------------------------------- */
//cstat -MISRAC2012-Rule-21.1
#include "main.h"
//cstat +MISRAC2012-Rule-21.1
#include "mc_type.h"
#include "mc_math.h"
#include "motorcontrol.h"
#include "regular_conversion_manager.h"
#include "mc_interface.h"
#include "digital_output.h"
#include "pwm_common.h"
#include "mc_tasks.h"
#include "parameters_conversion.h"
#include "mcp_config.h"
#include "mc_app_hooks.h"

/* USER CODE BEGIN Includes */
#include "as5048a_spd_pos_fdbk.h"
#include "as5048a_align_ctrl.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private define */


/* USER CODE END Private define */

/* Private variables -------------------------------------------------------- */
static volatile uint16_t hBootCapDelayCounterM1 = ((uint16_t)0);
static volatile uint16_t hStopPermanencyCounterM1 = ((uint16_t)0);

#define M1_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M1_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0.000 * ((uint32_t)PWM_PERIOD_CYCLES / 2U))
#define M2_CHARGE_BOOT_CAP_TICKS       (((uint16_t)SYS_TICK_FREQUENCY * (uint16_t)10) / 1000U)
#define M2_CHARGE_BOOT_CAP_DUTY_CYCLES (uint32_t)(0 * ((uint32_t)PWM_PERIOD_CYCLES2 / 2U))



/* USER CODE BEGIN Private Variables */
/* ── External sensor & alignment objects (defined in mc_config_common.c) ── */
extern AS5048A_SPD_Handle_t       AS5048A_SPD_M1;
extern AS5048A_AlignCtrl_Handle_t AS5048A_AlignCtrlM1;
/* USER CODE END Private Variables */

/* Private function prototypes ---------------------------------------------- */
void TSK_MediumFrequencyTaskM1(void);
void FOC_InitAdditionalMethods(uint8_t bMotor);
void FOC_CalcCurrRef(uint8_t bMotor);
void TSK_MF_StopProcessing(uint8_t motor);
MCI_Handle_t *GetMCI(uint8_t bMotor);
static uint16_t FOC_CurrControllerM1(void);
void TSK_SafetyTask_PWMOFF(uint8_t motor);

/* USER CODE BEGIN Private Functions */
/* USER CODE END Private Functions */

/**
  * @brief  Initialises the whole MC core according to user defined parameters.
  */
__weak void FOC_Init(void)
{
  /* USER CODE BEGIN MCboot 0 */
  /* USER CODE END MCboot 0 */

  /*********************************************************/
  /*    PWM and current sensing component initialization   */
  /*********************************************************/
  pwmcHandle[M1] = &PWM_Handle_M1._Super;
  R3_2_Init(&PWM_Handle_M1);

  /* USER CODE BEGIN MCboot 1 */
  /* USER CODE END MCboot 1 */

  /******************************************************/
  /*   PID component initialization: speed regulation   */
  /******************************************************/
  PID_HandleInit(&PIDSpeedHandle_M1);

  /******************************************************/
  /*   Main speed sensor: TAD2144 SPI+DMA               */
  /******************************************************/
  AS5048A_SPD_Init(&AS5048A_SPD_M1);

  /******************************************************/
  /*   Encoder alignment component (TAD2144 version)    */
  /******************************************************/
  AS5048A_EAC_Init(&AS5048A_AlignCtrlM1, pSTC[M1],
                    &VirtualSpeedSensorM1, &AS5048A_SPD_M1);

  /******************************************************/
  /*   Speed & torque component initialization          */
  /******************************************************/
  STC_Init(pSTC[M1], &PIDSpeedHandle_M1, &AS5048A_SPD_M1._Super);

  /********************************************************/
  /*   PID component initialization: current regulation   */
  /********************************************************/
  PID_HandleInit(&PIDIqHandle_M1);
  PID_HandleInit(&PIDIdHandle_M1);

  /*************************************************/
  /*   Power measurement component initialization  */
  /*************************************************/
  pMPM[M1]->pVBS     = &(BusVoltageSensor_M1._Super);
  pMPM[M1]->pFOCVars = &FOCVars[M1];

  pREMNG[M1] = &RampExtMngrHFParamsM1;
  REMNG_Init(pREMNG[M1]);

  FOC_Clear(M1);
  STC_Clear(pSTC[M1]);
  FOCVars[M1].bDriveInput = EXTERNAL;
  FOCVars[M1].Iqdref      = STC_GetDefaultIqdref(pSTC[M1]);
  FOCVars[M1].UserIdref    = STC_GetDefaultIqdref(pSTC[M1]).d;

  MCI_ExecSpeedRamp(&Mci[M1],
  STC_GetMecSpeedRefUnitDefault(pSTC[M1]), 0);

  /* USER CODE BEGIN MCboot 2 */
  /* USER CODE END MCboot 2 */
}

/**
 * @brief Stop processing helper.
 */
void TSK_MF_StopProcessing(uint8_t motor)
{
  R3_2_SwitchOffPWM(pwmcHandle[motor]);
  FOC_Clear(motor);
  STC_Clear(pSTC[motor]);
  TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
  Mci[motor].State = STOP;
}

/**
  * @brief  Medium-frequency periodic Motor Control task for Motor 1.
  */
__weak void TSK_MediumFrequencyTaskM1(void)
{
  /* USER CODE BEGIN MediumFrequencyTask M1 0 */
  /* USER CODE END MediumFrequencyTask M1 0 */

  int16_t wAux = 0;
  (void)AS5048A_SPD_CalcAvrgMecSpeedUnit(&AS5048A_SPD_M1, &wAux);
  PQD_CalcElMotorPower(pMPM[M1]);

  if (MCI_GetCurrentFaults(&Mci[M1]) == MC_NO_FAULTS)
  {
    if (MCI_GetOccurredFaults(&Mci[M1]) == MC_NO_FAULTS)
    {
      switch (Mci[M1].State)
      {

        case IDLE:
        {
          if ((MCI_START == Mci[M1].DirectCommand)
              || (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand))
          {
            if (pwmcHandle[M1]->offsetCalibStatus == false)
            {
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_START);
              Mci[M1].State = OFFSET_CALIB;
            }
            else
            {
              pwmcHandle[M1]->OffCalibrWaitTimeCounter = 1u;
              (void)PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC);
              R3_2_TurnOnLowSides(pwmcHandle[M1], M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
              Mci[M1].State = CHARGE_BOOT_CAP;
            }
          }
          else
          {
            /* FW stays in IDLE */
          }
          break;
        }

        case OFFSET_CALIB:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (PWMC_CurrentReadingCalibr(pwmcHandle[M1], CRC_EXEC))
            {
              if (MCI_MEASURE_OFFSETS == Mci[M1].DirectCommand)
              {
                FOC_Clear(M1);
                STC_Clear(pSTC[M1]);
                Mci[M1].DirectCommand = MCI_NO_COMMAND;
                Mci[M1].State = IDLE;
              }
              else
              {
                R3_2_TurnOnLowSides(pwmcHandle[M1], M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
                TSK_SetChargeBootCapDelayM1(M1_CHARGE_BOOT_CAP_TICKS);
                Mci[M1].State = CHARGE_BOOT_CAP;
              }
            }
            else
            {
              /* wait for calibration to finish */
            }
          }
          break;
        }

        case CHARGE_BOOT_CAP:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_ChargeBootCapDelayHasElapsedM1())
            {
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              FOCVars[M1].bDriveInput = EXTERNAL;
              STC_SetSpeedSensor(pSTC[M1], &VirtualSpeedSensorM1._Super);

              AS5048A_SPD_Clear(&AS5048A_SPD_M1);

              FOC_Clear(M1);

              if (AS5048A_EAC_IsAligned(&AS5048A_AlignCtrlM1) == false)
              {
            	AS5048A_EAC_StartAlignment(&AS5048A_AlignCtrlM1);
                Mci[M1].State = ALIGNMENT;
              }
              else
              {
                /* Already aligned (e.g. restart) – go straight to RUN */
                STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
                STC_SetSpeedSensor(pSTC[M1], &AS5048A_SPD_M1._Super);
                FOCVars[M1].bDriveInput = INTERNAL;
                FOC_InitAdditionalMethods(M1);
                FOC_CalcCurrRef(M1);
                STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]);
                MCI_ExecBufferedCommands(&Mci[M1]);
                Mci[M1].State = RUN;
              }
              PWMC_SwitchOnPWM(pwmcHandle[M1]);
            }
            else
            {
              /* wait for bootstrap charge */
            }
          }
          break;
        }

        case ALIGNMENT:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            bool isAligned = AS5048A_EAC_IsAligned(&AS5048A_AlignCtrlM1);
            bool EACDone   = AS5048A_EAC_Exec(&AS5048A_AlignCtrlM1);

            if ((isAligned == false) && (EACDone == false))
            {

                /* Apply d-axis current so rotor pulls toward the VSS angle */
                qd_t IqdRef;
                IqdRef.q = 0;
                IqdRef.d = STC_CalcTorqueReference(pSTC[M1]);
                FOCVars[M1].Iqdref = IqdRef;
            }
            else
            {
              /* Alignment done – prepare for RUN via WAIT_STOP_MOTOR */
              R3_2_SwitchOffPWM(pwmcHandle[M1]);
              STC_Clear(pSTC[M1]);
              STC_SetControlMode(pSTC[M1], MCM_SPEED_MODE);
              STC_SetSpeedSensor(pSTC[M1], &AS5048A_SPD_M1._Super);
              FOC_Clear(M1);
              R3_2_TurnOnLowSides(pwmcHandle[M1], M1_CHARGE_BOOT_CAP_DUTY_CYCLES);
              TSK_SetStopPermanencyTimeM1(STOPPERMANENCY_TICKS);
              Mci[M1].State = WAIT_STOP_MOTOR;
              /* USER CODE BEGIN MediumFrequencyTask M1 EndOfEncAlignment */
              /* USER CODE END MediumFrequencyTask M1 EndOfEncAlignment */
            }
          }
          break;
        }

        case RUN:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 2 */

            /* USER CODE END MediumFrequencyTask M1 2 */
            MCI_ExecBufferedCommands(&Mci[M1]);
            FOC_CalcCurrRef(M1);
          }
          break;
        }

        case STOP:
        {
          if (TSK_StopPermanencyTimeHasElapsedM1())
          {
            /* USER CODE BEGIN MediumFrequencyTask M1 5 */
            /* USER CODE END MediumFrequencyTask M1 5 */
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* wait for stop */
          }
          break;
        }

        case FAULT_OVER:
        {
          if (MCI_ACK_FAULTS == Mci[M1].DirectCommand)
          {
            Mci[M1].DirectCommand = MCI_NO_COMMAND;
            Mci[M1].State = IDLE;
          }
          else
          {
            /* stay in FAULT_OVER */
          }
          break;
        }

        case FAULT_NOW:
        {
          Mci[M1].State = FAULT_OVER;
          break;
        }

        case WAIT_STOP_MOTOR:
        {
          if (MCI_STOP == Mci[M1].DirectCommand)
          {
            TSK_MF_StopProcessing(M1);
          }
          else
          {
            if (TSK_StopPermanencyTimeHasElapsedM1())
            {
              AS5048A_SPD_Clear(&AS5048A_SPD_M1);
              FOCVars[M1].bDriveInput = INTERNAL;   /* ← without this the speed PI never outputs */
              R3_2_SwitchOnPWM(pwmcHandle[M1]);
              FOC_InitAdditionalMethods(M1);
              STC_ForceSpeedReferenceToCurrentSpeed(pSTC[M1]);
              MCI_ExecBufferedCommands(&Mci[M1]);
              FOC_CalcCurrRef(M1);
              Mci[M1].State = RUN;
            }
            else
            {
              /* Nothing to do */
            }
          }
          break;
        }

        default:
          break;
       }
    }
    else
    {
      Mci[M1].State = FAULT_OVER;
    }
  }
  else
  {
    Mci[M1].State = FAULT_NOW;
  }
  /* USER CODE BEGIN MediumFrequencyTask M1 6 */
  /* USER CODE END MediumFrequencyTask M1 6 */
}

/**
  * @brief  Re-initialises current/voltage variables, clears PID controllers.
  */
__weak void FOC_Clear(uint8_t bMotor)
{
  /* USER CODE BEGIN FOC_Clear 0 */
  /* USER CODE END FOC_Clear 0 */

  ab_t NULL_ab = {0, 0};
  qd_t NULL_qd = {0, 0};
  alphabeta_t NULL_alphabeta = {0, 0};

  FOCVars[bMotor].Iab        = NULL_ab;
  FOCVars[bMotor].Ialphabeta = NULL_alphabeta;
  FOCVars[bMotor].Iqd        = NULL_qd;
  FOCVars[bMotor].Iqdref     = NULL_qd;
  FOCVars[bMotor].hTeref     = 0;
  FOCVars[bMotor].Vqd        = NULL_qd;
  FOCVars[bMotor].Valphabeta = NULL_alphabeta;
  FOCVars[bMotor].hElAngle   = 0;

  PID_SetIntegralTerm(pPIDIq[bMotor], 0);
  PID_SetIntegralTerm(pPIDId[bMotor], 0);

  PWMC_SwitchOffPWM(pwmcHandle[bMotor]);

  /* USER CODE BEGIN FOC_Clear 1 */
  /* USER CODE END FOC_Clear 1 */
}

/**
  * @brief  Initialise additional methods in START_TO_RUN state.
  */
__weak void FOC_InitAdditionalMethods(uint8_t bMotor) //cstat !RED-func-no-effect
{
  if (M_NONE == bMotor)
  {
    /* Nothing to do */
  }
  else
  {
    /* USER CODE BEGIN FOC_InitAdditionalMethods 0 */
    /* USER CODE END FOC_InitAdditionalMethods 0 */
  }
}

/**
  * @brief  Compute Iqdref from the speed controller.
  */
__weak void FOC_CalcCurrRef(uint8_t bMotor)
{
  qd_t IqdTmp;

  __disable_irq();
  IqdTmp = FOCVars[bMotor].Iqdref;
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 0 */
  /* USER CODE END FOC_CalcCurrRef 0 */

  if (INTERNAL == FOCVars[bMotor].bDriveInput)
  {
    FOCVars[bMotor].hTeref = STC_CalcTorqueReference(pSTC[bMotor]);
    IqdTmp.q = FOCVars[bMotor].hTeref;
  }

  __disable_irq();
  FOCVars[bMotor].Iqdref = IqdTmp;
  __enable_irq();

  /* USER CODE BEGIN FOC_CalcCurrRef 1 */
  /* USER CODE END FOC_CalcCurrRef 1 */
}

/* ═══════════════════════════════════════════════════════════════════════════ */
/*  HIGH-FREQUENCY TASK (15 kHz – ADC ISR context)                           */
/* ═══════════════════════════════════════════════════════════════════════════ */

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  FOC high-frequency task – executes every PWM cycle.
  */
__weak uint8_t FOC_HighFrequencyTask(uint8_t bMotorNbr)
{
  uint16_t hFOCreturn;
  /* USER CODE BEGIN HighFrequencyTask 0 */
  /* USER CODE END HighFrequencyTask 0 */

  RCM_ReadOngoingConv();
  RCM_ExecNextConv();

  /* ── AS5048A: consume previous DMA result + kick off next one ────────── */
  (void)AS5048A_SPD_CalcAngle(&AS5048A_SPD_M1);

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_1 */
  /* USER CODE END HighFrequencyTask SINGLEDRIVE_1 */

  hFOCreturn = FOC_CurrControllerM1();

  /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_2 */
  /* USER CODE END HighFrequencyTask SINGLEDRIVE_2 */

  if (hFOCreturn == MC_DURATION)
  {
    MCI_FaultProcessing(&Mci[M1], MC_DURATION, 0);
  }
  else
  {
    /* USER CODE BEGIN HighFrequencyTask SINGLEDRIVE_3 */
    /* USER CODE END HighFrequencyTask SINGLEDRIVE_3 */
  }

  return bMotorNbr;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__((section (".ccmram")))
#endif
#endif
/**
  * @brief  Core FOC current controller for Motor 1.
  */
inline uint16_t FOC_CurrControllerM1(void)
{
  qd_t Iqd, Vqd;
  ab_t Iab;
  alphabeta_t Ialphabeta, Valphabeta;
  int16_t hElAngle;
  uint16_t hCodeError = MC_NO_FAULTS;
  SpeednPosFdbk_Handle_t *speedHandle;

  speedHandle = STC_GetSpeedSensor(pSTC[M1]);
  hElAngle    = SPD_GetElAngle(speedHandle);
  PWMC_GetPhaseCurrents(pwmcHandle[M1], &Iab);
  Ialphabeta  = MCM_Clarke(Iab);
  Iqd         = MCM_Park(Ialphabeta, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    Vqd.q = PI_Controller(pPIDIq[M1],
                          (int32_t)(FOCVars[M1].Iqdref.q) - Iqd.q);
    Vqd.d = PI_Controller(pPIDId[M1],
                          (int32_t)(FOCVars[M1].Iqdref.d) - Iqd.d);
  }
  else
  {
    Vqd.q = 0;
    Vqd.d = 0;
  }

  Vqd        = Circle_Limitation(&CircleLimitationM1, Vqd);
  Valphabeta = MCM_Rev_Park(Vqd, hElAngle);

  if (PWMC_GetPWMState(pwmcHandle[M1]) == true)
  {
    hCodeError = PWMC_SetPhaseVoltage(pwmcHandle[M1], Valphabeta);
  }

  FOCVars[M1].Vqd        = Vqd;
  FOCVars[M1].Iab        = Iab;
  FOCVars[M1].Ialphabeta = Ialphabeta;
  FOCVars[M1].Iqd        = Iqd;
  FOCVars[M1].Valphabeta = Valphabeta;
  FOCVars[M1].hElAngle   = hElAngle;

  return hCodeError;
}

/* USER CODE BEGIN mc_task 0 */
/* USER CODE END mc_task 0 */

/******************* (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
