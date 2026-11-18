# Logolith

![MCU](https://img.shields.io/badge/MCU-STM32F405RG-03234B)
![Language](https://img.shields.io/badge/language-C11-blue)
![Drivers](https://img.shields.io/badge/drivers-STM32%20LL%20only-informational)
![RTOS](https://img.shields.io/badge/RTOS-none-lightgrey)
![License](https://img.shields.io/badge/license-MIT-green)

**Logolith** is the flight controller firmware of **Aerolith**, a custom cinelifter built to carry a full-frame mirrorless camera (Sony A7S III class) for cinematic yet dynamic aerial footage.
Portfolio : https://lemonochrme.github.io

Like the rest of Aerolith — frame, ESCs and flight controller — Same philosophy, Logolith is written from scratch rather than forked from Betaflight, ArduPilot or PX4. Every driver, filter and control loop is meant to be understood, measured and justified, not copied.

> ⚠️ **Early stage — work in progress.** Logolith does not fly yet. Sensor and RC drivers are done; the control loops are being built now. See [Status](#status).

---

## Table of contents

- [Goals](#goals)
- [Hardware](#hardware)
- [Firmware architecture](#firmware-architecture)
- [Design rules](#design-rules)
- [Status](#status)
- [IMU validation results](#imu-validation-results)
- [Roadmap](#roadmap)
- [Building and flashing](#building-and-flashing)
- [License](#license)

---

## Goals

- **Stable, predictable flight over state-of-the-art agility.** A cinelifter carrying a camera needs a smooth, well-damped attitude response and robust failure handling, not freestyle-level stick feel.
- **Understand every layer.** Bare-metal C on STM32 **LL** drivers. No HAL, no RTOS, no third-party flight stack. CubeMX is used only to generate peripheral init code.
- **Built for LithCore, portable by design.** The primary target is LithCore, the custom STM32F405 flight controller of Aerolith, but board-specific code is kept as much as possible separate from drivers and control logic so Logolith can be brought up on other F4-class boards and maybe on a future F7 based redesign of LithCore.
- **Measure before tuning.** Each sensor is validated on the bench (noise, bias, self-test) before it is trusted by a control loop, and filters are placed from measured spectra (blackbox), not guessed.
- **Rate-agnostic code.** No loop frequency is hard-coded: filters and controllers take `fs` / `dt`, and their discrete coefficients are computed at init. This keeps the door open to a different loop rate or a faster MCU.
- **Borrow architecture, not code.** Betaflight is the reference for the fast loop (gyro filter chain, D-term chain, feedforward, mixer chain); ArduPilot is the reference for everything above the rate loop (position → velocity → acceleration → attitude cascade, EKF-style estimation, prioritized motor mixing, coaxial X8).

---

## Hardware

### Aerolith airframe

| Item | Choice |
| --- | --- |
| Frame | Custom carbon cinelifter, 380 mm diagonal, designed in Fusion 360 |
| Configuration | Quad-X first, coaxial X8 later |
| Motors | 3115, 640 KV |
| Propellers | carbon 9050 tri-blade |
| Battery | 4S LiPo (bench / development), 6S (final) |
| ESC | Generic 4in1 50A 30.5x30.5mm ESC (bench / development), individual Custom 60A Aerolith ESC (final) |
| Onboard cam system | RunCam Phoenix 2 SP V3 with AT7456ELAH OSD and RUSHFPV RUSH Tank III Ultimate 30.5x30.5mm, DJI O4 Air Unit mounting holes provided on the chassis (planned for future F7 based custom FC with additional UARTs) |
| Payload | Mirrorless camera, A7S III class |

### LithCore flight controller

| Function | Part | Interface | Firmware status |
| --- | --- | --- | --- |
| MCU | STM32F405RGT6, 168 MHz (8 MHz HSE + PLL), Cortex-M4F | — | ✅ |
| IMU | ICM-42688-P (two possible configs : mounted onboard or on a damped standalone external board, the long term goal is to do archive a precise thermal regulation of the IMU on the stand alone board to archive minimal drift and maximal precision) | SPI1 @ 21 MHz, DMA2, INT2 → EXTI | ✅ |
| Barometer | LPS22HH | SPI3 (shared with the OSD) | in progress |
| Optical flow | PMW3901 | SPI2, MOTION → EXTI1 | (planned for future F7 based custom FC) |
| Rangefinder | VL53L1X (height above ground for optical flow) | I2C | (planned for future F7 based custom FC) |
| Blackbox | W25Q128 SPI flash (16 MB) | SPI2 | in progress |
| OSD | AT7456ELAH | SPI3 | in progress |
| RC link | ExpressLRS 2.4 GHz, CRSF @ 420 kbaud | UART, DMA circular + IDLE | ✅ |
| GPS + compass | u-blox M10 module with integrated magnetometer (QMC5883L / IST8310 class), mast-mounted away from power wiring | UART4 (GNSS), I2C (compass) | planned |
| Motor outputs | OneShot125 | TIM1, TIM3, TIM8 | X4 version ✅, X8 version in progress, (RPM filtering planned for future F7 based custom FC with additional UARTs) |

Power: 5 V rail from an LMR51430 buck, split 3V3 digital / analog rails, and a dedicated low-noise LDO (TPS7A2033) for the IMU.

The RC transmitter is also custom (STM32F411 + ExpressLRS module) and lives in its own repository.

---

## Firmware architecture

### Control structure

```mermaid
flowchart LR
    RX["ELRS receiver<br/>CRSF 420 kbaud"] -->|"UART DMA + IDLE"| RCP["RC processing<br/>deadband · expo · LPF"]
    ICM["ICM-42688-P<br/>SPI1 + DMA2"] -->|"UI_DRDY"| GYR["Gyro chain<br/>bias · notch · low-pass"]
    GYR --> AHRS["Gated Mahony<br/>attitude estimate"]
    RCP -->|"attitude setpoint"| ANG["Angle loop<br/>quaternion error"]
    AHRS --> ANG
    ANG -->|"rate setpoint"| RATE["Rate PID<br/>D on measurement · FF"]
    GYR --> RATE
    RATE --> MIX["Mixer chain<br/>limit · matrix · prioritized desat."]
    MIX --> ESC["Motor output<br/>OneShot125 / DShot"]
```

A classic cascade:

- **Inner loop — rate PID.** Triggered by the IMU data-ready interrupt, so the loop is hardware-timed by the sensor rather than by a free-running timer. D term computed on the measurement only (no derivative kick), with its own filter chain. Conditional-clamping anti-windup and I-term relax. Loop rate: see [Loop rate](#loop-rate).
- **Outer loop — angle.** The attitude error is computed in SO(3), not by subtracting Euler angles: `q_err = q_current⁻¹ ⊗ q_setpoint`, giving the error in the body frame. The small-angle vector part (`2·q_err.xyz`) feeds a P(I) controller that outputs rate setpoints.
- **Mixer.** A chain rather than a single matrix: PID sum limit → Quad-X (later X8 coaxial) matrix → prioritized desaturation → thrust linearization → output window [floor, ceiling]. See [Signal-flow principles](#signal-flow-principles).

### Signal-flow principles

1. **Each block has its own rate.** Every shared signal has one producer, a documented update rate, and an explicit consistency mechanism (double buffer + flag, or a sub-microsecond critical section).
2. **Setpoint and measurement travel separate paths**, and the error is only formed at the summing junction. D acts on the measurement with its own filters; feedforward acts on the derivative of the shaped setpoint. Setpoint tracking (feedforward + shaping) and disturbance rejection (P / I / D) are therefore tuned independently. For smooth footage, setpoint shaping comes first.
3. **Variable gains are computed one iteration ahead.** Gains that depend on throttle, such as dynamic cutoffs or attenuation, are computed in iteration n−1. This breaks the algebraic loop between PID and mixer and keeps the loop execution time deterministic.
4. **Prioritized desaturation**: roll / pitch first, then thrust, then yaw last. Betaflight-style airmode is deliberately not used. On a multi-kilogram coaxial X8, yaw authority is weak and saturates first, and raising throttle to preserve it would make the aircraft climb.
5. **Injection points for the upper layers** (yaw rate, roll / pitch angle, throttle) are planned from the start, so that altitude, position and navigation can plug in without touching the fast loop.
6. **Two consumers, two gyro filters.** The control loop wants minimum delay; the attitude estimator wants bias-free, alias-free integration and gets its own gyro filtering.

Racing-oriented features (launch control, throttle boost, crash recovery, FPV camera mix) are out of scope.

### Attitude estimation

Logolith uses a **single-IMU** estimator: an adaptive Mahony filter running on the ICM-42688-P.

- **Gated accelerometer correction.** Accelerometer feedback is weighted down when `‖a‖` departs from the reference norm measured at boot (≈ 1.025 g uncalibrated on the current board), so manoeuvres and vibration do not pull the horizon.
- **Integral bias tracking.** The Mahony `Ki` term tracks the residual gyro bias in flight, on top of the boot calibration. This works on roll and pitch only, where gravity gives an absolute reference; the yaw bias is unobservable without the compass of the GNSS module.

A BNO085 fusion chip was evaluated and dropped: running two different MEMS devices in an inner/outer cascade introduces coherence errors (different biases, scale factors and mounting) that cannot be tuned away, plus extra latency, for a marginal accuracy gain over a well-gated single-IMU filter.

### Gyro filtering

```
ICM-42688-P (inside the chip):
ADC → decimation 32 kHz → factory notch → anti-alias filter → user offset → UI filter → ODR

Firmware:
gyro → bias tracking → notch(es) → low-pass → rate PID (P, I)
                                └→ D-term notch / low-pass → rate PID (D)
```

- **Inside the sensor.** The anti-alias filter runs at 32 kHz, before decimation to the output data rate, so the chip can decimate cleanly in one place. It is currently set to ≈ 1 kHz (3 dB). The factory notch only targets the MEMS sense resonance (1–3 kHz) and is useless against motor noise. **Motor harmonic rejection is therefore done in software.**
- **Expected spectrum.** The 640 KV tri-blade props spin at ≈ 126 Hz at full throttle on 4S (estimate, to be measured). Each arm is a pair of 4 mm beams 30° apart, so each blade crosses two obstacles per turn. The resulting pulse pattern repeats every 120°: lines are expected at 3×, 9× and 12× the rotation frequency (12× reinforced), while 6× should cancel. This is to be confirmed by blackbox spectrograms.
- **Folded lines.** At a 1 kHz loop rate, high harmonics fold below Nyquist, and the signature of a folded line is a frequency that *falls* as throttle rises. Folding is deterministic, so once motor RPM is known, a notch can be placed on the computed folded frequency.
- **Now, without RPM telemetry**: low-pass filtering plus blackbox measurement, then either a throttle-indexed notch (from a bench RPM vs throttle curve) or a home-made dynamic notch.
- **Later, with bidirectional DShot**: a per-motor RPM filter on the 1× and 3× harmonics (per motor, because on a coaxial X8 the lower props spin faster in the upper props' wash).

### Loop rate

**The fast loop runs at 1 kHz.** It was first brought up at the IMU's 8 kHz output data rate. Now the ICM decimates internally: ODR 1 kHz, anti-alias filter at 213 Hz running at 32 kHz before decimation, and a 2nd-order UI filter at 493 Hz (0.7 ms group delay). The rate moves to 2 kHz only if folded motor lines show up in the logs. Why not 8 kHz:

- **Little phase to gain.** Digital loop delay is about 1.5 sample periods: 1.5 ms at 1 kHz, 0.19 ms at 8 kHz, so only ~5° of phase difference at 10 Hz. On a 9" cinelifter, the phase budget is dominated by the motor + propeller time constant (estimated 30–60 ms, to be measured, i.e. a crossover around 5–15 Hz) and by the filtering.
- **No extra information.** With an anti-alias filter at a few hundred hertz, an 8 kHz stream is mostly oversampled, already-filtered data.
- **Real costs**:
  - OneShot125 pulses (125–250 µs) cannot run above ~4 kHz;
  - the discrete D term amplifies white noise by the sample rate;
  - an 8 kHz × 40-byte blackbox stream would saturate the SPI flash;
  - biquads at low cutoff / high `fs` have poles very close to the unit circle.

A faster F7 / H7 target would decouple the gyro rate (notches and decimation at 2–4 kHz) from the PID rate (1 kHz), like Betaflight does.

### Execution model

Bare-metal, interrupt-driven, no RTOS. The CPU budget on a 168 MHz Cortex-M4F is a few percent, so prioritized interrupts are enough.

- **Sensor → DMA → callback.** A data-ready EXTI starts a DMA burst; the DMA transfer-complete interrupt parses the data and runs the loop. Nothing in the fast path polls.
- **NVIC priorities** follow time-criticality: IMU EXTI and its DMA first (both at the same preemption level, so a new data-ready can never restart a read whose completion handler is still running), then RC link, then slow sensors, telemetry and debug. SysTick (1 ms timebase) sits at the lowest priority.
- **Shared data between loops** (e.g. the 3-float rate setpoint written by the outer loop and read by the inner loop) is exchanged through a double buffer or a short critical section, since a multi-word copy is not atomic on Cortex-M4.

---

## Design rules

These rules come from bring-up issues that actually happened on this project.

1. **Every configuration write is read back and logged at init.** Silent failures are real: a wrong full-scale setting once left the gyro at ±250 dps while the code assumed ±2000 dps (8× error), and a wrong register address once disabled the anti-alias filter.
2. **Interrupts are armed last.** EXTI and DMA interrupts stay disabled (NVIC off, pending flags cleared) until sensor init is complete, so a sensor asserting INT early cannot trigger a half-configured handler.
3. **DMA buffers live in SRAM, never CCM.** CCM RAM is not reachable by the DMA controllers on the STM32F4.
4. **Do not trust eye-closed generated LL init.** CubeMX does not enable the DMA TC interrupt or the SPI peripheral itself; these are done explicitly in the drivers.
5. **The fast-loop sensor owns its bus.** The IMU is alone on SPI1; slower sensors go elsewhere (the barometer sits on SPI3 with the OSD).
6. **Verify the silicon.** A counterfeit ICM-42688-P (wrong WHO_AM_I, flat single-bank register map) was found and replaced. Critical ICs are sourced from authorized distributors.
7. **`LL_Init1msTick()` does not enable the SysTick interrupt.** It only starts the counter for `LL_mDelay()` polling. Without an explicit `LL_SYSTICK_EnableIT()`, the millisecond counter stayed at 0 and every time-based feature (slow loop, RC failsafe, calibration timeouts) silently never ran.
8. **Never output a dangerous command at boot.** ESC range calibration (full throttle) is a separate, explicit bench procedure; init only outputs the stop pulse, and every motor command defaults to 0.
9. **No blocking debug output in interrupts.** Console prints are blocking (~11 µs per character), so every periodic print runs from the main loop in idle time. The ISR only takes a snapshot, and the main loop prints it at ~2 Hz. Prints are paused during calibrations.

---

## Status

Legend: ✅ done and verified on hardware · 🔧 in progress · 📋 planned

| Module | Status | Notes |
| --- | --- | --- |
| System clock, debug/flash by STLink V3, USB DFU Flashing| ✅ | 168 MHz from 8 MHz HSE |
| ICM-42688-P driver | ✅ | Multi-bank register map (banks 0/1/2/4), low-noise mode, gyro ±2000 dps and accel ±16 g both @ 1 kHz ODR (decimated inside the chip), gyro anti-alias filter 213 Hz, UI_DRDY on INT2, 13-byte DMA burst. A single `ICM42688_ODR_HZ` constant drives every rate-dependent setting. A lost DMA transfer is detected and restarted |
| ICM-42688-P self-test | ✅ | MEMS self-test passes on gyro and accel |
| Boot gyro calibration | ✅ | Bias measured at rest and written to the sensor's `OFFSET_USER` registers (~0.6 s, board still) |
| Accel calibration | ✅ | An accelerometer calibration (6 poses calibration) is needed on every new mounted chassis to suppress mounting related physical offsets and in sensors naturals offsets. Verified on the bench: started from the radio (SA+SD held 2 s, throttle low), faces detected automatically in any order, bias + full 3×3 scale/cross-axis matrix, sanity checks, applied to every IMU sample in the DMA callback |
| Persistent config storage | ✅ | Verified on the bench (calibration reloaded after reboot): flash sector 11 reserved (the link fails if the firmware grows into it), append-only log with CRC32, survives reflashing (except a full-chip erase) |
| 1 ms timebase (SysTick) | ✅ | SysTick interrupt enabled (see design rule 7), confirmed on hardware (measured IMU rate consistent with the ICM clock tolerance) |
| ELRS / CRSF receiver driver | ✅ | DMA circular buffer + IDLE interrupt, 3-state parser, CRC8 (DVB-S2), RC channels / link statistics / device frames, arming gated on link quality |
| Gated Mahony | 🔧 | |
| Gyro / D-term filtering | 📋 | Low-pass first; notches placed from blackbox spectra (expected lines 3×, 9×, 12× rotation frequency) |
| Fast loop timing | ✅ | Runs at the end of the IMU DMA interrupt, hardware-timed by the sensor; real `dt` from the DWT cycle counter. Measured at 1 kHz: 996 Hz (ICM clock 0.4 % slow), period jitter ±0.1 µs, whole IMU chain 10.3 µs average / 11.2 µs max, i.e. 1.0 % CPU with the loop still empty. Gyro noise at rest visibly lower than at 8 kHz (anti-alias filter 997 → 213 Hz) |
| Rate PID + Quad-X mixer | 📋 | Target: stable ACRO flight before ANGLE |
| Motor output (OneShot125, then DShot) | OneShot125 ✅, DShot 🔧 | OneShot125 pulses are fired by the fast loop itself (TIM8 one-pulse mode), so the output follows the IMU clock with a fixed delay instead of beating against a free-running 1 kHz timer; if the loop stops, the pulses stop and the ESCs cut the motors. ESC range calibration is a separate explicit procedure. Board pads M1..M8 mapped to TIM8 / TIM1 / TIM3 in the CubeMX config. Motor positions and spin directions verified on the bench with a radio-triggered one-by-one spin test (props off), quad-X props-out. ESC range calibration ✅ from the radio (disarmed, throttle high, SA+SD 2 s: motors pulse at 1 Hz as a props-on warning until throttle is lowered, then throttle passthrough; SA+SD or failsafe exits). Motors now respond over the full 0–100 % range; before it, the ESCs only responded between ≈ 15 % and ≈ 83 % (BLHeli default 1148/1832 µs ÷ 8) |
| Arming, failsafe | ✅ | Verified on the bench: arm switch SF (latching), armed on a rising edge only (switch left on at boot or when the link comes back must be cycled first), refused if link quality is low, throttle is not low, or an accelerometer calibration is running or missing; disarmed on switch off or link timeout. Motor outputs do not depend on it yet |
| Angle loop (quaternion error) | 📋 | |
| Barometer + altitude hold | 📋 | Complementary filter baro / vertical acceleration |
| Optical flow position hold | 📋 | PMW3901 + VL53L1X |
| Blackbox (SPI flash) | 📋 | Needed for tuning: raw / filtered gyro, setpoint, P / I / D / F, motor outputs, throttle, Vbat. W25Q128 on SPI2 (shared with the PMW3901) |
| OSD | 📋 | |
| GPS | 📋 | |
| Coaxial X8 support | 📋 | |

---

## IMU validation results

Measured on LithCore, board at rest on a desk, genuine TDK ICM-42688-P (WHO_AM_I = `0x47`).

| Quantity | Result | Comment |
| --- | --- | --- |
| Gyro noise | ≈ 0.08–0.10 °/s RMS | ~0.4 °/s RMS before the LDO fix |
| Accel noise | ≈ 0.5 mg RMS | |
| Raw gyro bias | ≈ −11 / −1.9 / −8.3 °/s (X/Y/Z) | Above the typical datasheet figure, which is not production-tested. Identical in dps at ±250 and ±2000 dps full scale → genuine analog zero-rate offset, not a configuration artifact |
| Short-term bias stability | ±0.01 °/s | Session-to-session delta ≈ 0.3 °/s |
| Residual after boot calibration | ≈ 0.2 °/s | Post-power-up drift. Roll/pitch: absorbed by the Mahony `Ki` term. Yaw: ≈ 12–18 °/min heading drift, negligible in ACRO / ANGLE (yaw is rate-controlled), corrected by the compass once fitted |
| Accel norm at rest | ≈ 1.021–1.026 g | Z zero-g offset at the datasheet limit; corrected by a 6-position calibration on the frame, applied in software |
| Self-test | PASS (gyro and accel) | |

Accelerometer offsets are deliberately **not** calibrated at boot: doing so would absorb the real tilt of the drone on the ground.

---

## Roadmap

### Immediate next steps

1. **ESC output bench check** at 1 kHz (pulses now fired by the loop): ESC arming, motor test, ESC calibration.
2. **Gyro filtering:** low-pass first, notches placed from measured spectra.
3. **Rate PID + mixer chain:** separate P / I, D and feedforward paths; PID sum limit, prioritized desaturation; stop pulse on every output whenever `Security_IsArmed()` is false.
4. **Blackbox** (needed for tuning): raw and filtered gyro, setpoint, P / I / D / F per axis, motor outputs, throttle, battery voltage.
5. **Bench measurements:** RPM vs throttle (acoustic spectrum of the blade-pass frequency), motor time constant (throttle step), frame resonances.

### Known TODOs

- **Motor cut on disarm:** the future rate loop / mixer must output the stop pulse whenever `Security_IsArmed()` is false.
- **Motor mapping:** pad ↔ timer channel is confirmed (M1..M4 = PC6..PC9 = TIM8 CH1..CH4, M5..M7 = PA8..PA10 = TIM1, M8 = PB1 = TIM3), motor positions and spin directions verified on the bench, viewed from above with the camera at the front (M1 rear left CW, M2 front left CCW, M3 rear right CCW, M4 front right CW: quad-X props-out, see `esc_driver.c`). ESC range calibrated (125–250 µs): motors now respond over the full 0–100 % range.
- **ESC update rate:** one OneShot125 pulse per loop iteration (~1 kHz); DShot (bidirectional, with eRPM) comes with the Aerolith ESCs.
- **Motor pole pairs:** to confirm (7 is typical for a 3115, 12N14P) before converting eRPM to Hz.
- **ESC_M8 timer scale:** TIM3 is clocked from APB1, so its timer runs at 12 MHz instead of 24 MHz and its pulse values are half those of M1..M7.

### Phases

**Phase 1 — Sensing and link** *(done)*
ICM-42688-P driver, boot calibration, CRSF receiver, accelerometer 6-face calibration with flash storage.

**Phase 2 — ACRO flight** *(next)*
Loop rate set to 1 kHz (done, bench check pending), gyro and D-term filtering, rate PID with separate P / I, D and feedforward paths, mixer chain with prioritized desaturation, blackbox, bench measurements (RPM vs throttle, motor time constant, spectrum). The drone must be stable in ACRO before anything else.

**Phase 3 — ANGLE flight**
Gated Mahony, quaternion error in body frame, angle loop. Hover tests.

**Phase 4 — Altitude and position**
LPS22HH on SPI3 (bus shared with the OSD), altitude hold, then optical-flow position hold with PMW3901 + VL53L1X, then GPS + compass (u-blox M10, UBX `NAV-PVT`) with magnetometer calibration.

**Phase 5 — Tooling**
OSD, CLI for in-field gain tuning (the blackbox moves up to Phase 2, since tuning depends on it).

**Phase 6 — Aerolith final configuration**
- **ESCs**: bidirectional DShot with the Aerolith ESCs, giving a per-motor RPM filter and dynamic idle.
- **6S**: re-tuning, thrust linearization, voltage compensation, and a check of folded lines at full throttle.
- **Airframe**: coaxial X8 mixing (8×4 matrix, yaw from the upper/lower split on each arm).
- **Controller**: F7 / H7 port, with an isolated board layer and gyro rate decoupled from PID rate if useful.

---

## Building and flashing

### Requirements

- Visual Studio Code with the **STM32Cube for VS Code** extension (CMake + Arm GNU toolchain)
- An ST-Link (SWD), or a USB-C cable for DFU

### Build

Open the folder in VS Code and build with the extension, or from a terminal:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

### Flash

- **SWD:** ST-Link on the SWD header, flash and debug from VS Code.
- **USB DFU:** hold BOOT0 while plugging in USB-C, then flash with STM32CubeProgrammer App or use the script "flash_DFU.sh" made by me (with love) directly in your terminal (you will need STM32CubeProgrammer CLI installed), with this command:

```sh
./flash_DFU.sh
```

Debug outputs are enabled / disabled per subsystem with the `debug_*` flags in `global.c` (register readback, IMU data ~2 Hz, RC link quality and channels ~2 Hz, fast loop timing ~1 Hz, warnings) and are printed on the debug UART (UART5, 921600 baud).

---

## License

[MIT](LICENSE)
