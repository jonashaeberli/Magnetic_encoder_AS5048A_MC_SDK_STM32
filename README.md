# AS5048A SPI Encoder Integration for ST Motor Control SDK (MC SDK 6.4.1)

A custom **absolute magnetic encoder** speed/position feedback driver that integrates the
**AMS AS5048A** (14-bit SPI magnetic rotary encoder) into the **STMicroelectronics Motor
Control SDK** FOC firmware, running on an **STM32G431RB** (NUCLEO-G431RB).

The AS5048A driver is a *drop-in replacement* for the SDK's built-in quadrature-encoder
(`encoder_speed_pos_fdbk`) speed/position sensor. It reads the rotor angle over **SPI + DMA**
inside the high-frequency FOC loop and feeds true absolute mechanical/electrical angle and
speed to the field-oriented control regulators.

---

## Why this project

The ST MC SDK ships with sensorless, Hall, and incremental quadrature-encoder feedback
options, but no driver for SPI absolute encoders such as the AS5048A. Absolute encoders are
attractive for gimbal/robotics-style PMSM motors (here a **GM3506** gimbal motor, 11
pole-pairs) because they give a known rotor position immediately after a single alignment,
with no index pulse and no quadrature counting.

This repository contains a hand-written `AS5048A_SPD_Handle_t` feedback component plus a
matching alignment controller, wired into the SDK task scheduler so the rest of the FOC stack
(speed/torque PI loops, state machine, MCP telemetry) runs unmodified.

---

## Hardware

| Item | Detail |
|------|--------|
| MCU | STM32G431RBT3 (Cortex-M4F, 170 MHz, CCM RAM used) |
| Control board | NUCLEO-G431RB |
| Power board | X-NUCLEO-IHM16M1 (3-shunt, internal op-amps) |
| Motor | GM3506 gimbal PMSM, 11 pole-pairs |
| Encoder | AMS AS5048A, 14-bit absolute, SPI |
| Current sensing | 3-shunt, ADC1/ADC2 |
| Comms | UART2 / ASPEP (ST MC Workbench Motor Pilot) |

### AS5048A wiring (SPI3)

| Signal | STM32 pin | Notes |
|--------|-----------|-------|
| SPI3_SCK  | PC10 (AF6) | |
| SPI3_MISO | PC11 (AF6) | |
| SPI3_MOSI | PC12 (AF6) | |
| NSS (chip select) | PA15 | Software-controlled GPIO (`AS5048A_NSS`) |
| DEBUG (scope/timing) | PD2 | Optional debug toggle |

SPI3 configuration: **Master, Mode 1 (CPOL=0, CPHA=1, 2-edge), 8-bit, MSB-first,
prescaler /16**. The AS5048A uses 16-bit command/response frames sent as two 8-bit bytes.

DMA: **DMA2 Channel 1 = SPI3_RX**, **DMA2 Channel 2 = SPI3_TX** (priority below the FOC ADC
ISR so motor control timing is never disturbed).

---

## Software architecture

### Custom files (the actual integration work)

| File | Role |
|------|------|
| [`Inc/as5048a_spd_pos_fdbk.h`](mainV3_sensor_AS5048A/Inc/as5048a_spd_pos_fdbk.h) / [`Src/as5048a_spd_pos_fdbk.c`](mainV3_sensor_AS5048A/Src/as5048a_spd_pos_fdbk.c) | AS5048A SPI+DMA speed & position feedback component. Builds on the SDK `SpeednPosFdbk_Handle_t` base class. |
| [`Inc/as5048a_align_ctrl.h`](mainV3_sensor_AS5048A/Inc/as5048a_align_ctrl.h) / [`Src/as5048a_align_ctrl.c`](mainV3_sensor_AS5048A/Src/as5048a_align_ctrl.c) | Rotor-to-sensor alignment controller (mirror of the SDK `enc_align_ctrl`, adapted to the absolute sensor). |

### SDK files touched to wire it in (inside `USER CODE` sections)

| File | Change |
|------|--------|
| [`Src/mc_config_common.c`](mainV3_sensor_AS5048A/Src/mc_config_common.c) | Declares `AS5048A_SPD_M1` and `AS5048A_AlignCtrlM1` handle instances (SPI handle, CS pin, FIFO depth, pole-pair ratio, reliability limits). |
| [`Src/mc_tasks_foc.c`](mainV3_sensor_AS5048A/Src/mc_tasks_foc.c) | Calls `AS5048A_SPD_Init` / `AS5048A_EAC_Init` at boot, runs `AS5048A_SPD_CalcAngle` in the **high-frequency task**, `AS5048A_SPD_CalcAvrgMecSpeedUnit` in the medium task, and replaces encoder alignment in the state machine. |
| [`Src/stm32g4xx_it.c`](mainV3_sensor_AS5048A/Src/stm32g4xx_it.c) | DMA2 Ch1/Ch2 IRQ handlers + `HAL_SPI_TxRxCpltCallback` route completion to `AS5048A_SPD_DmaCompleteCallback`. |
| [`Src/main.c`](mainV3_sensor_AS5048A/Src/main.c) / `Src/stm32g4xx_hal_msp.c` | `MX_SPI3_Init`, SPI3 GPIO/DMA MSP, and `AS5048A_NSS` GPIO output configuration. |

> The project was generated in **ST MC Workbench 6.4.1** with the speed sensor set to
> *Quadrature Encoder*, then the encoder driver was swapped for the AS5048A component. This
> keeps all Workbench-generated scaffolding (state machine, alignment flow, MCP registers)
> intact while substituting the actual sensor.

### Data flow

```
 High-freq FOC ISR ──► AS5048A_SPD_CalcAngle()
        │                  ├─ read previous DMA result (RxBuf → 14-bit RawAngle)
        │                  ├─ convert to s16 mechanical/electrical angle (+offset)
        │                  ├─ accumulate wrap-safe multi-turn angle
        │                  └─ start next SPI+DMA READ_ANGLE frame
        ▼
 DMA2 Ch1 IRQ ──► HAL_SPI_TxRxCpltCallback ──► AS5048A_SPD_DmaCompleteCallback()
        │                  └─ de-assert CS, set DmaComplete flag
        ▼
 Medium-freq task ──► AS5048A_SPD_CalcAvrgMecSpeedUnit()
                           └─ FIFO of angle deltas → averaged mechanical speed + el. DPP
```

### Key design points

- **Pipelined SPI reads.** The AS5048A returns the angle requested by the *previous* frame,
  so `Init` primes the pipeline with a blocking READ_ANGLE; thereafter each FOC tick consumes
  the last DMA result and immediately launches the next, fully overlapping SPI with control.
- **Odd-parity command frames.** `as5048a_build_cmd()` sets the read bit and computes the
  AS5048A odd-parity bit for every 16-bit command word.
- **Power-on diagnostics.** `Init` does blocking reads of `ERRFL`, `DIAG+AGC`, `MAG` and
  `ANGLE` and sets `DiagnosticFlags` (parity error, magnetic field too high/low, CORDIC
  overflow, offset-compensation not ready). These feed the SDK reliability check.
- **Absolute alignment.** The alignment controller drives the rotor to a fixed electrical
  angle via the virtual speed sensor + d-axis current, then records the AS5048A reading as the
  mechanical offset (`AS5048A_SPD_SetMecAngle`). After one alignment the absolute position is
  known on every restart.
- **Wrap-safe speed/position.** Mechanical angle wraps at ±32768 (s16degree); both the
  multi-turn accumulator and the speed estimator fold single-sample deltas larger than half a
  revolution back into range.
- **CCM RAM.** `AS5048A_SPD_CalcAngle` is placed in CCM RAM (when enabled) to keep it off the
  flash bus during the FOC ISR.

---

## Repository layout

```
mainV3_sensor_AS5048A/
├─ mainV3_sensor_AS5048A.stwb6        # ST MC Workbench 6.4.1 project
└─ mainV3_sensor_AS5048A/
   ├─ mainV3_sensor_AS5048A.ioc       # STM32CubeMX configuration
   ├─ Inc/                            # Headers (incl. as5048a_*.h)
   ├─ Src/                            # Sources (incl. as5048a_*.c)
   ├─ Drivers/                        # STM32G4 HAL + CMSIS (generated)
   ├─ MCSDK_v6.4.1-Full/              # MC SDK middleware (generated)
   ├─ ftl/                            # Workbench code-generation templates
   └─ STM32CubeIDE/                   # CubeIDE project, linker script, Debug/
```

---

## Building & flashing

1. Install **STM32CubeIDE** (1.16+) and, optionally, **X-CUBE-MCSDK / ST MC Workbench 6.4.1**.
2. Open the project:
   - **STM32CubeIDE:** *File → Open Projects from File System…* and select the
     `mainV3_sensor_AS5048A/mainV3_sensor_AS5048A/STM32CubeIDE` folder, **or**
   - **ST MC Workbench:** open `mainV3_sensor_AS5048A.stwb6` to inspect/regenerate.
3. Build (`Project → Build`) and flash to the NUCLEO-G431RB via the on-board ST-LINK.
4. Connect the AS5048A to SPI3 per the wiring table above.
5. Monitor / spin the motor with the **Motor Pilot** tool over UART (ASPEP).

> ⚠️ If you regenerate code from ST MC Workbench, the AS5048A wiring lives in
> `USER CODE` sections and in the dedicated `as5048a_*` files, so it survives regeneration —
> but always re-check `mc_tasks_foc.c`, `mc_config_common.c` and `stm32g4xx_it.c` after a
> regen.

---

## Status & notes

- Target motor: GM3506 gimbal PMSM, 11 pole-pairs, nominal current 0.81 A, PWM 12 kHz,
  max application speed ~1446 rpm.
- This is a working integration intended as a reference for adding SPI absolute encoders to
  the ST MC SDK; tune motor/PI parameters in ST MC Workbench for your own hardware before
  running.
- Most files under `Drivers/`, `MCSDK_v6.4.1-Full/` and `STM32CubeIDE/Debug/` are
  generated/third-party and are covered by their original ST licenses.

## License

The custom `as5048a_*` integration code in this repository is provided as-is for reference.
STMicroelectronics HAL, CMSIS, and MC SDK components retain their original ST license terms
(see the headers of the respective files). Add your own license file before publishing if you
intend to relicense the custom portions.
