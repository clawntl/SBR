# Balance controller: implementation spec

Spec for building the controller firmware for a two-wheeled balancing robot: Waveshare ESP32-S31-Zero, LSM6DSV16X IMU, two MiniFOC Mk2 drivers on CAN. Written for Claude Code. The human-readable overview (with diagrams) is the doc *ESP32-S31 balancing robot controller — implementation plan*; if the two disagree, ask.

Last updated 2026-11-25.

---

## 0. Ground rules (read first)

1. **Work one stage at a time** (§10). Don't start a stage until the previous stage's acceptance checks pass and the user has confirmed. Each stage ends with something the user can test on hardware.
2. **Don't add features nobody asked for.** This robot is a sister project to [EBS-SBR](https://github.com/engbuildspace/EBS-SBR). EBS-SBR is reference material only. Don't copy its features, settings or behaviours unless the user has agreed (§11 lists the candidates). That includes "small" ones such as learned trim, lift detection, captive portals or settings registries. If something seems needed, ask.
3. **Ask before:**
   - changing the pin map, the CAN IDs or the protocol;
   - adding a dependency;
   - changing a decided item (§1.3);
   - writing any code path that can command non-zero motor current outside the stage that introduces it.
4. **Motor safety.** Every current command goes through the safety gate (§7.6) and a per-wheel clamp. Test commands that drive motors clamp to 0.5 A and only exist from Stage 4. The balance controller drives motors only from Stage 7.
5. **Pin every version:** platform release, libraries (tag or commit SHA), vendored code with its licence.
6. **Open decisions** (§12): if a stage needs one that is still open, stop and ask.

---

## 1. System

### 1.1 Hardware

| Part | Role | Notes |
|---|---|---|
| Waveshare ESP32-S31-Zero | Controller | ESP32-S31: dual-core RISC-V at 320 MHz, FPU, 2 × TWAI-FD controllers, USB Serial/JTAG. 16/32 MB flash, 16/32 MB PSRAM |
| LSM6DSV16X | IMU, on the controller protoboard **near the top of the body** (not on the axle) | SPI. INT1 only |
| SIT1051T/3 | CAN transceiver on the controller protoboard | Must be the **/3 (VIO) version**: VCC 5 V, VIO 3.3 V, so RXD swings 0–3.3 V (ESP32 pins are not 5 V tolerant). S tied low = normal mode. A plain SIT1051T has no VIO and outputs 5 V on RXD: then RXD needs a divider |
| 2 × MiniFOC Mk2 | FOC current-mode motor drivers | [BLDC-Driver](https://github.com/clawntl/BLDC-Driver) firmware, node 1 = left, node 2 = right |
| 2 × GM3506 | Direct-drive gimbal motors | 11 pole pairs, 5.6 Ω line-to-line (2.8 Ω per phase) |
| Battery | 3S 18650 Li-ion | 5 V buck on the controller protoboard into the Zero's 5V pin |
| Battery sense (later) | INA226 on I²C (GPIO50/51) | Not on the protoboard yet; don't implement until asked |

The robot has **two rings in the pitch plane** (one each side, outboard of the wheels; see the user's CAD). Their centre is close to the centre of mass, **well above the wheel axle**. Consequences:

- **On its wheels** (near upright) the robot is an ordinary inverted pendulum: the CoM sits a distance l ≈ d_c above the axle, where d_c is the axle-to-ring-centre distance.
- **On its rings** (every other pose, including upside down) the wheels are in the air and work as reaction wheels. Gravity pulls only weakly, through the small CoM offset ε from the ring centre: the robot settles where the CoM is lowest, and holding any other pose needs a small steady torque.
- **Self-righting** means rolling on the rings from a pose such as 90° (lying on its front or back) up onto the wheels. That's ring mode plus the hand-over to wheel mode (Stage 9).

The firmware must balance upright on the wheels, hold poses on the rings (upside down included), and eventually move between the two (§7).

### 1.2 Test tools the user has

- **CAN Bench** ([clawntl/can-bench](https://github.com/clawntl/can-bench)), on a CANable 2.5. CAN terminal and device dashboard driven by TOML profiles; periodic transmit; Rhai test sequences with PASS/FAIL (`--sequence "<name>" --exit` runs one from the command line). `profiles/bldc-driver.toml` already decodes the MiniFOC protocol in both directions. Silent (listen-only) mode for sniffing. Bus load, error counters, frame-rate and gap monitors.
- **IMU-Visualiser** ([clawntl/IMU-Visualiser](https://github.com/clawntl/IMU-Visualiser)). Reads serial lines `w;x;y;z` (quaternion mode) or `roll;pitch;yaw` in degrees (Euler mode); separators `;`, `,` or whitespace. Lines that don't parse are shown in its console and otherwise ignored, so log lines can share the port. Has axis remap, tare, CSV record and playback.

### 1.3 Decided

| Topic | Decision |
|---|---|
| Framework | Arduino-ESP32 4.0 on PlatformIO (pioarduino platform, ESP-IDF 6.1 underneath); IDF APIs where timing matters |
| Loop rate | **960 Hz**: one control cycle per IMU sample (the IMU's ODR steps are 480/960/1920 Hz; a 1000 Hz loop would beat against the IMU) |
| Controller hardware | One protoboard near the top of the body: Zero, IMU, CAN transceiver, 5 V buck |
| CAN library | [ACAN-ESP32FD](https://github.com/clawntl/ACAN-ESP32FD), commit `b955866f07ebe36f57f9880af0c3bbf7963197ae` |
| CAN bus | Classic CAN 2.0, 11-bit IDs, 1 Mbit/s, 87.5 % sample point |
| IMU interrupt | INT1 only: FIFO watermark into GPIO46. INT2 not used |
| Attitude estimation | **Our own filter** on raw gyro and accel (§6). The on-chip SFLP is only a cross-check, using its gravity-vector output, never its quaternion |
| Controller | Wheel mode: one LQR on 6 states (wheel position, tilt, heading and their rates) with 2 inputs (common and differential current). Ring mode: its own small controller using the wheels as reaction wheels. Driving is by set points, never by injecting currents (§7) |
| Cores | Control on core 1. Everything else, including a future web page, on core 0 (§8) |

---

## 2. Pin map

Header pins on the Zero: left edge (top to bottom) 5V, GND, 3V3, GPIO0, 1, 2, 3, 4, 5. Right edge (top to bottom) GPIO58 (U0TXD), 59 (U0RXD), 51, 50, 49, 48, 47, 46, 6. Strapping pins (36, 37, 60, 61) are not on the header.

```cpp
// include/PinConfig.h
#pragma once
#include <driver/gpio.h>

// IMU, LSM6DSV16X on SPI (through the GPIO matrix; <= 10 MHz)
constexpr gpio_num_t PIN_IMU_CS   = GPIO_NUM_0;
constexpr gpio_num_t PIN_IMU_SCK  = GPIO_NUM_1;
constexpr gpio_num_t PIN_IMU_MOSI = GPIO_NUM_2;   // IMU SDI
constexpr gpio_num_t PIN_IMU_MISO = GPIO_NUM_3;   // IMU SDO
constexpr gpio_num_t PIN_IMU_INT1 = GPIO_NUM_46;  // FIFO watermark, rising edge

// CAN0 to the transceiver
constexpr gpio_num_t PIN_CAN_TX   = GPIO_NUM_4;
constexpr gpio_num_t PIN_CAN_RX   = GPIO_NUM_5;

// On the Zero itself
constexpr gpio_num_t PIN_RGB_LED  = GPIO_NUM_60;  // WS2812B-1010 (strapping pin: output only after boot)
constexpr gpio_num_t PIN_BOOT_BTN = GPIO_NUM_61;  // active low (strapping pin: don't hold at reset)

// Spare / optional (unused until the user decides; see §12)
// GPIO48: spare.
// GPIO50 / GPIO51: I2C SDA / SCL, reserved for a future INA226 battery monitor.
// GPIO47 / GPIO49: spare; CAN1 TX / RX if a second bus is ever used.
// GPIO6: spare, slow signals only (next to the antenna). GPIO58/59: UART0, leave free.
```

Electrical notes:

- **GPIO0/1** are the 32 kHz crystal pins, but no crystal is fitted.
- **GPIO58/59** are restricted in the GPIO matrix, and the ROM log comes out on 58.
- **5V pin:** VBUS reaches it through a Schottky, so a battery buck can feed it with USB connected.
- **IMU wiring:** the IMU is on the same protoboard as the Zero. Run SPI at the IMU's maximum, 10 MHz (check the datasheet's figure); the GPIO matrix handles that easily.

---

## 3. Toolchain and project layout

- **Framework (decided):** PlatformIO with a pioarduino platform release that ships Arduino-ESP32 4.0.x on ESP-IDF 6.1. Arduino 4.0.0 (2026-11-19) includes an `esp32s31` board. Pin the platform to a release URL, never to `stable`. ESP32-S31 is a **preview target** in IDF 6.1.
- **Board:** if PlatformIO has no Waveshare S31 Zero board, add `boards/waveshare_esp32s31_zero.json` based on the `esp32s31` dev module: flash size per variant, PSRAM enabled, USB CDC on boot.
- **Build flags:** `-DARDUINO_USB_CDC_ON_BOOT=1`, `-DCORE_DEBUG_LEVEL=2`.
- **Environments:** `s31zero` (firmware) and `native` (host unit tests, Unity).
- **Use IDF APIs directly where timing matters:** `spi_master`, GPIO ISR service, FreeRTOS tasks and notifications, `esp_timer`. Use Arduino only for convenience (Serial, NVS `Preferences` if wanted).
- **Dependencies (nothing else without asking):**
  - ACAN-ESP32FD at the commit in §1.3.
  - ST [lsm6dsv16x-pid](https://github.com/STMicroelectronics/lsm6dsv16x-pid), vendored into `lib/lsm6dsv16x/` at commit `5e4b2f214590c4d108554517f895b30eb0ca93bb`, with its BSD-3 `LICENSE`.

```
firmware/
  platformio.ini
  boards/                 board JSON if needed
  include/PinConfig.h  include/Config.h (compile-time constants, versions)
  lib/lsm6dsv16x/         vendored ST driver + LICENSE
  src/
    main.cpp              task creation only; loop() empty
    imu/      Lsm6dsv16x.{h,cpp}   SPI glue, config, FIFO drain, tag decode, half-float
    est/      GravityFilter.{h,cpp} (§6), Odometry.{h,cpp}
    can/      CanBus.{h,cpp} (ACAN wrapper), MiniFoc.{h,cpp} (protocol, link state)
    control/  Controller.{h,cpp} (§7), GainTable.{h,cpp}, SafetyGate.{h,cpp}, BalanceState.{h,cpp}
    app/      Console.{h,cpp}, Settings.{h,cpp} (NVS), Telemetry.{h,cpp}, StatusLed.{h,cpp}
    util/     RingBuffer.h (SPSC), Snapshot.h (seqlock), Wrap.h, Half.h
  test/                   native unit tests
tools/
  telemetry.py            USB binary stream -> CSV
  model/                  robot model, LQR gain-table design, margin checks -> include/GainTable.inc
  can-bench/              robot.toml profile + sequences (fake drivers etc.)
docs/implementation-plan.md   this file
```

---

## 4. CAN and the MiniFOC protocol

### 4.1 Bus setup (ACAN-ESP32FD)

```cpp
ACAN_ESP32FD_Settings s(1000UL * 1000UL, 875);   // 1 Mbit/s, 87.5 %
s.mTxPin = PIN_CAN_TX;  s.mRxPin = PIN_CAN_RX;
s.mFilters.addMaskFilter(0x100, 0x7F0, false);   // driver telemetry 0x100-0x10F (check the return value)
const uint32_t err = can0.begin(s);              // call from a task pinned to core 1 (ISR lands there)
```

- Log `err`, the actual bit rate and the sample point at boot. Treat a non-zero `err` as fatal for motor use.
- For self-tests use `s.mModuleMode = ACAN_ESP32FD_Settings::LOOP_BACK_NO_ACK`.
- Receive with non-blocking `receive()`: frames are copied out of the ISR into a software FIFO (default 32 deep; raise it if `driverReceiveFIFODidOverflow()` is ever true). Send with `tryToSend()`.
- Monitor `busErrorCount()`, `isBusOff()`, `txErrorCounter()` and `rxErrorCounter()`. On bus-off: log it, mark both drivers Offline (forces 0 A in the gate), and call `recoverFromBusOff()` from the supervisor, not the control task.
- The S31 has 3 mask filters and 1 range filter. Only one mask filter is used now.

### 4.2 Frames (BLDC-Driver protocol 7, firmware ≥ 1.7)

Always target the latest BLDC-Driver; checked against firmware 1.7.1 (2026-11-25).

Floats are IEEE-754 **big-endian**. Node *n* ∈ 1..15; left = 1, right = 2 by default, configurable.

| ID | Dir | Layout |
|---|---|---|
| `0x010 + (n−1)/2` | ctrl → drv | 8 bytes: odd node's current f32 in bytes 0–3, even node's in bytes 4–7. **NaN = no command** for that node. Clamped by the driver to ±2 A. **Send at least every 100 ms** or the driver applies 0 A |
| `0x020 + n` | ctrl → drv | `[cmd][payload…]` |
| `0x100 + n` | drv → ctrl | `[cmd][payload…]` |

Driver → controller, on `0x100 + n` (check the length; ignore extra bytes):

| cmd | Payload |
|---|---|
| `0x02` velocity | f32 shaft rad/s; then (protocol 6, 8 bytes) i16 measured q current in mA, and i8 measured d current in 16 mA steps. Interval set by `PARAM_VELOCITY_INTERVAL_MS` (default 10 ms) |
| `0x03` status (every 100 ms) | u8 state (1 IDLE, 2 RUNNING, 3 FAULT); u8 flags (bit 0 COMMAND_TIMEOUT, 1 CALIBRATION_USED, 2 INVERTED, 3 DRIVER_FAULT); u16 supply mV (0 = none); i16 input mA (`0x8000` = none, MiniFOC only, 7-byte frame) |
| `0x10` start reply | f32 1.0 ok / 0.0 failed |
| `0x20` info reply | protocol, fw major, minor, patch, image KB |
| `0x22` set-param reply | param, status (0 OK), f32 value now |
| `0x27` board info reply | page 0: u32 capabilities, hw rev; pages 1–5: model name chars |
| `0xFE` NACK | command, reason |
| `0xFF` error | f32 code (1 motor init, 2 current sense, 3 FOC init, 4 driver fault) |

Controller → driver commands used, on `0x020 + n`:

- `[0x10]` Start FOC. Blocks the driver about 0.3 s with a stored alignment, a few seconds without (sensor alignment), then it replies. The start timeout is 5 s (§4.3).
- `[0x20]` GET_INFO.
- `[0x27][page]` GET_BOARD_INFO.
- `[0x22][0x10][f32 ms]` SET_PARAM velocity interval. Runtime only: re-send after every driver reset.

**Never send** SAVE (`0x24`), REBOOT (`0x26`) or node/invert changes while balancing; both stop the motor. Driver configuration is done with CAN Bench, not from this firmware, unless the user asks.

**Anti-cogging** (`CMD_COGGING` `0x29`, `PARAM_COG_COMP`, protocol 7) is configured and calibrated with CAN Bench only; this firmware never sends them. A calibration aborts if a current command above 50 mA arrives, so calibrate with the controller off the bus or in `Disabled` (it sends 0 A).

Full reference: [can-protocol.md](https://github.com/clawntl/BLDC-Driver/blob/main/docs/can-protocol.md).

### 4.3 Driver link (per node)

States: `Offline → Idle → Starting → Running`, plus `Fault`.

| From | To | When |
|---|---|---|
| any | Offline | No status frame for 300 ms |
| Offline | Idle / Running / Fault | Status frame with that state |
| Idle | Starting | Start requested (Stage 4: console command; automatic policy is an open decision, §11) |
| Starting | Running | Start reply 1.0 |
| Starting | Fault | Start reply 0.0, or no reply in 5 s |
| Running | Idle | Status says IDLE (driver reset), then re-send the velocity interval |

The control task reads link states from a snapshot. Only `Running` on **both** nodes lets current through the gate.

### 4.4 Signs

Set each driver's `invert` (with CAN Bench) so that, for **both** wheels:

- positive current gives positive shaft velocity;
- positive shaft velocity is the wheel turning about body +y relative to the body.

Body +y points to the robot's **left** (§5.3), so positive velocity rolls the robot forward when it is upright. Firmware then uses one sign convention for both wheels.

### 4.5 Bus load

At 960 Hz control (one `0x010` frame per cycle) with 1 ms velocity telemetry from both drivers, the bus carries about 3 k frames/s, roughly 40–50 % load with bit stuffing. That's acceptable, and the current frame has the lowest ID, so it always wins arbitration. Measure it in Stage 4 with CAN Bench. If accessories are added later, put them on CAN1 rather than this bus.

---

## 5. IMU (LSM6DSV16X)

### 5.1 SPI and configuration

- **SPI:** 4-wire, mode 3 (ST parts accept modes 0 and 3; confirm in the datasheet), at the IMU's maximum clock (10 MHz; confirm in the datasheet). Read = address | 0x80. Drive CS high before any other SPI traffic. WHO_AM_I (`0x0F`) must read `0x70`.
- **Use the vendored ST driver** through its `stmdev_ctx_t` read/write callbacks.

Configuration order:

1. Software reset, then wait for it to complete.
2. BDU on, auto-increment on.
3. Accel ±8 g. The IMU is high on the body and sees large accelerations; revisit after Stage 3 logs.
4. Gyro ±2000 dps.
5. Accel and gyro ODR 960 Hz, high-performance mode.
6. SFLP on at 480 Hz with **gravity vector only** batched to the FIFO (tag `0x17`). Not the game rotation vector: half-float precision fails near inverted. Not the gyro bias.
7. FIFO in continuous mode, batching accel (tag `0x02`) and gyro (tag `0x01`) at 960 Hz, plus timestamps.
8. FIFO watermark = 2 words (1 gyro + 1 accel = one 960 Hz cycle). INT1 = FIFO threshold, push-pull, active high. INT2 unused.

### 5.2 Read path

1. GPIO46 rising-edge ISR (IRAM) calls `vTaskNotifyGiveFromISR` for the control task.
2. The control task reads FIFO_STATUS for the word count.
3. It burst-reads all words: 7 bytes each, tag + 6 data.
4. It dispatches by tag. Each accel/gyro pair is one estimator update with dt from the timestamps: normally exactly one pair per cycle.
5. The newest SFLP gravity sample is kept for the cross-check.
6. An overrun or more than 2 cycles of data at once is counted and logged (from core 0).

Half-floats are only needed if the game vector is ever enabled for the visualiser. Keep a tested `half_to_float` in `util/Half.h` anyway.

### 5.3 Frames and conventions

- **Body frame:** x forward, y along the axle to the **left**, z up (right-handed). A config matrix `R_sb` maps sensor axes to body axes; it's found in Stage 2 with IMU-Visualiser.
- **Tilt θ:** rotation about +y. θ > 0 means the top of the body is forward of the axle. Upright θ = 0, inverted θ = ±180°.
- **Up vector:** **u** is the unit vector "world up" expressed in body axes. At rest the accelerometer reads +1 g along **u**.
  - θ = atan2(−u_x, u_z)
  - side lean λ = asin(u_y)
- **Lever arms:** `r_imu` is the IMU position relative to the axle centre, in body axes, in metres. The ring centre is at `(0, 0, d_c)` from the axle, so the IMU relative to the ring centre is `r_imu − (0, 0, d_c)`. Both come from CAD (config) and are checked in Stage 3.

---

## 6. Estimator

### 6.1 Gravity filter

A Mahony-style filter on the unit up-vector, at the 960 Hz IMU rate. No quaternion and no Euler angles, so there is no singularity anywhere. The state is **u** (unit 3-vector) and gyro bias **b** (3-vector).

Per IMU sample (ω_m rad/s, f_m m/s², dt s):

1. ω = ω_m − b
2. **Predicted non-gravity specific force at the IMU**:
   - f_pred = a_axle + α × r_imu + ω × (ω × r_imu)
   - α = angular acceleration, from successive gyro samples through a first-order low-pass (cut-off a config value, start at 40 Hz).
   - The reference point depends on the contact mode (§7.1):
     - **Wheel mode:** reference point = axle; r = r_imu. a_ref = v̇·h + v·ψ̇·(u × h): axle acceleration along h plus turning centripetal. v = r_w·(φ̇_rel,mean + θ̇) is the ground speed from the wheels; v̇ is v differentiated and low-passed.
     - **Ring mode:** reference point = ring centre; r = r_imu − (0, 0, d_c). The rings roll without slipping, so a_ref = R·θ̈·h, with θ̈ = α_y. The centre moves R·θ̇ along h.
     - Near the mode boundary use the mode the controller is in.
   - f_pred above uses that a_ref in place of a_axle, and that r in place of r_imu.
   - h = normalize(ŷ × u) is the horizontal direction perpendicular to the axle. It's undefined when the axle is vertical: skip the correction there.
3. **Measured up direction:** û = normalize(f_m − f_pred).
4. **Accel trust:** w = clamp(1 − | ‖f_m − f_pred‖ − g | / (0.2 g), 0, 1).
5. **Error and bias:**
   - e = û × u
   - ω_c = ω + k_p·w·e
   - b ← b − k_i·w·e·dt (clamp |b| ≤ 0.1 rad/s per axis)
6. **Propagate:** u ← normalize(u − dt·(ω_c × u))

Starting gains, to tune in Stage 3: k_p = 1.0 rad/s, k_i = 0.01 rad/s². Bias along **u** (yaw) is unobservable from gravity. Leave it unless the user wants a still-detector to learn it.

### 6.2 Outputs (published every control cycle)

| Output | Definition |
|---|---|
| tilt θ | atan2(−u_x, u_z), range (−π, π] |
| tilt rate θ̇ | ω_y (bias-corrected gyro, no filter correction term) |
| side lean λ | asin(clamp(u_y, −1, 1)) |
| turn rate ψ̇ | ω · u (about world vertical; right when tilted or inverted) |
| heading ψ | ∫ψ̇ dt, continuous float, reset to 0 on engage |
| wheel angle φ | mean of the two shaft angles (each integrated from its velocity), plus θ: wheel rotation relative to the world |
| wheel speed φ̇ | mean shaft velocity + θ̇ |
| pose valid | false when |u_y| > sin(60°): axle too close to vertical, θ meaningless |
| SFLP cross-check | θ_sflp from the SFLP gravity vector; difference logged, never used for control |

### 6.3 Unit tests (`native`)

- Static poses at θ = 0, ±10, ±90, ±179, 180°, with side lean 0 and 20°: θ error < 0.05° after settling.
- Constant rotation about y: tracking error < 0.1°.
- Gyro bias of 0.02 rad/s on y: converges and θ error < 0.2° after 30 s.
- **Lever-arm tests:** (a) wheel mode, body swinging about the axle (θ = 10°·sin(2π·1.5 t)); (b) ring mode, rolling on the rings (θ = 40°·sin(2π·0.8 t)). Use placeholder r_imu, d_c and R until the user gives the real ones. With compensation, peak θ error < 0.5°. Report the same figure with compensation off.
- Wrap: θ crossing ±180° produces no jump in wrap(θ − θ_ref).
- `half_to_float`: normals, subnormals, ±0, ±inf, NaN.

---

## 7. Controller

### 7.1 Geometry and contact modes

Config values from CAD (the user will supply them, §12):

| Value | Meaning |
|---|---|
| R | Ring radius (rolling surface) |
| r_w | Wheel radius |
| d_c | Axle to ring centre along body +z (positive: ring centre above the axle) |
| b | Track width |

The CoM is at the ring centre plus a small unknown offset ε.

The wheels touch the ground when their lowest point is at or below the ring's: d_c·cos θ + r_w ≥ R. That holds for |θ| ≤ θ_c = acos((R − r_w)/d_c), on the upright side only. If (R − r_w)/d_c ≥ 1 the wheels never protrude: stop and ask.

| Mode | Pose | Plant | Controller |
|---|---|---|---|
| **W** (wheels) | \|θ\| < θ_c − hyst | Two-wheeled inverted pendulum, CoM l = d_c + ε above the axle | 6-state LQR, §7.2 |
| **R** (rings) | \|θ\| > θ_c + hyst (upside down included) | Body rolls on the rings; wheels in the air act as reaction wheels; near-neutral | 3-state controller, §7.3 |

- Between the two thresholds the mode doesn't change (hysteresis, config, start at 2°).
- Also cross-check contact from behaviour. In mode W, wheel speed and body motion agree with rolling on the wheels; with the wheels in the air they don't. Log disagreements; don't act on them until reviewed.
- **Until Stage 9 implements mode R, entering mode R trips the gate (0 A).**

### 7.2 Wheel mode: model and LQR (tools/model, Python)

**Planar model:**
- Body mass m_b, CoM at l above the axle, inertia I_b about its CoM around the axle direction.
- Two wheels: m_w and I_w each (rotor included), radius r_w.
- Torque per wheel τ = K_t·i, reacting on the body. Viscous plus Coulomb friction from the spin test.

**Yaw model:** track b; yaw inertia about world vertical (depends on θ; small range here); differential wheel force gives the yaw torque.

**States:** x = [φ, θ, ψ, φ̇, θ̇, ψ̇]. φ is the mean wheel angle relative to the world. The controller acts on the error x − x_ref (§7.4); θ errors are wrapped to ±π.

**Inputs:** u = [i_sum, i_diff]; i_L = i_sum − i_diff, i_R = i_sum + i_diff, so i_diff > 0 turns left. Verify that sign in Stage 8.

**Design:**
- Discrete LQR at Ts = 1/960 s, with one sample of input delay. Either do a one-step state prediction feeding a 2×6 K, or use a 2×8 K on the states plus the previous input; choose in Stage 6 by margins, default prediction.
- K comes out block-structured for a symmetric robot ((φ, θ) to i_sum, ψ to i_diff). Keep the full 2×6 matrix in firmware.
- **Scheduling:** mode W only spans |θ| < θ_c. Check margins at θ₀ = 0 and at ±θ_c. Add table entries (5° steps) only if a single gain set fails the margins.
- **Margins:** at least 6 dB gain margin and 30° phase margin, loop broken at the plant input, with l at ± its uncertainty and K_t ±20 %.
- **Feed-forward:** also export N_θ and N_u (§7.4): the steady-state lean and current per unit of acceleration, from the same model.
- **Output:** `include/GainTable.inc`, with design parameters and the git hash in a comment.

### 7.3 Ring mode: model and controller

**Model:**
- The body and rings roll without slipping, so the ring centre moves R·θ̇ along h. Pose and position are tied: rolling 2πR turns the robot over once.
- Gravity torque is m·g·ε·sin θ, small.
- The wheels are free rotors: motor torque τ = K_t·i accelerates them and pushes back on the body.
- Equations of motion: (I_body+rings about contact + m·R²)·θ̈ = −2τ_eff + m·g·ε·sin θ, and I_w·(θ̈ + φ̈_rel) = τ. Derive exactly in tools/model.

**States and input:**
- States: [θ − θ_ref, θ̇, φ̇_rel] (mean wheel speed relative to the body).
- Input: i_sum. i_diff only holds the left/right wheel speed difference near 0: there is no yaw authority in ring mode.

**Design:**
- LQR (or pole placement) per pose if ε makes it pose-dependent; otherwise one gain set.
- φ̇_rel is a *weighted* state: holding a pose against a steady disturbance (ε, a slope) needs steady torque. The wheels then spin up, and momentum can only be shed by letting the robot roll. Report the wheel-speed drift rate at the measured ε, and trip at a wheel-speed limit (config, below the motors' top speed).

### 7.4 References and driving (set points, never injected currents)

A drive command never goes into the controller as a current or a disturbance. It shapes the references; the controller tracks them: u = u_ff − K·(x − x_ref).

**Mode W:**
1. Commands: ground speed v_cmd and turn rate ψ̇_cmd. Rate-limit them to a_max and a yaw-acceleration limit (config). That gives a_ref(t), v_ref(t) and ψ̇_ref(t).
2. φ̇_ref = v_ref / r_w and φ_ref = ∫φ̇_ref dt. ψ_ref = ∫ψ̇_ref dt.
3. **Lean and feed-forward** from the model's steady state under constant acceleration:
   - θ_ref = θ_trim + N_θ·a_ref, and θ̇_ref = N_θ·ȧ_ref.
   - u_ff = [N_u·a_ref + friction(v_ref), friction_yaw(ψ̇_ref)].
   - Without these, a speed change makes the robot fall behind its reference before it leans.
4. **Anti-windup:** clamp |φ − φ_ref| and |ψ − ψ_ref| to config limits. When clamped, move the reference, not the state, so a long push doesn't store up error.
5. **Engage:** φ_ref = φ and ψ_ref = ψ, so starting is bumpless.

**Mode R:** the command is a target pose θ_target (e.g. 180°) or a roll distance converted to Δθ = Δx / R, ramped at a rate limit.

**Mode changes (Stage 9):**
- Wheels to rings: ramp θ_ref past θ_c, then hand over to mode R.
- Rings to wheels: roll to just inside θ_c, then hand over to mode W with φ_ref = φ and ψ_ref = ψ.
- Plan the flip trajectory with the model before implementing it, and review it with the user.

### 7.5 Control law and limits

1. Mode W: i = u_ff − K·(x − x_ref). Mode R: i_sum = −K_R·(x_R − x_R,ref).
2. Per-wheel clamp to I_max (config, ≤ 2 A; start at 0.8 A).
3. Through the safety gate.

What happens when one wheel saturates (priority between balance and position terms) is a user decision (§11). Default: plain per-wheel clamp.

### 7.6 Balance state machine

| State | Meaning |
|---|---|
| `Disabled` | Default at boot and after `stop`; output 0 A |
| `Waiting` | Pose within the engage window of the selected target (mode W: \|θ\| < 10°; mode R: within 10° of θ_target), slow, held for 0.3 s |
| `Active` | Controller of the current mode drives the wheels |
| `Tripped` | A cut-out fired; needs `Waiting` again, or `stop` |

### 7.7 Safety gate (forces 0 A on both wheels)

The gate is the only path to the CAN current frame. It forces 0 A when any of these hold:

- either driver not Running;
- no IMU data for more than 3 cycles;
- pose invalid (axle near vertical: tipped over sideways);
- mode W and |wrap(θ − θ_ref)| > cut-out (default 25°, and never beyond θ_c + hyst);
- mode R entered before Stage 9 implements it;
- mode R and wheel speed above its limit (§7.3);
- |λ| > side cut-out (default 30°);
- console `stop` or BOOT pressed;
- state ≠ Active (except Stage 4 test commands);
- CAN bus-off.

Runaway (in mode W), lifted-wheel and battery cut-outs are **not** included until the user agrees (§11). Mention them when Stage 7 starts.

---

## 8. Firmware architecture

### 8.1 Tasks

| Task | Core | Priority | Runs | Does |
|---|---|---|---|---|
| `control` | 1 | configMAX_PRIORITIES − 2 | On INT1 notification, 960 Hz | FIFO drain → estimator → drain CAN RX → controller → safety gate → send `0x010` → push telemetry sample → publish snapshot |
| `supervisor` | 0 | 5 | 50 Hz | Driver link states, start requests, velocity-interval re-send, bus-off recovery, LED |
| `console` | 0 | 3 | On serial input | Text commands; queues requests to `control` |
| `telemetry` | 0 | 2 | Drains ring buffer | Binary stream over USB; decimated text streams for IMU-Visualiser |
| `web` (later) | 0 | ≤ 4 | | Only after the user asks |

### 8.2 Rules (the web page depends on them)

1. **Interrupt cores.** Interrupts are allocated on the core that installs them. Init SPI, the GPIO ISR service and `can0.begin()` from a short-lived task pinned to core 1. Init Wi-Fi (later) from core 0.
2. **Flash stalls.** Flash erase/write stalls both cores, for tens of ms per sector. So:
   - The ISR and the control path go in IRAM (`IRAM_ATTR`; verify with the map file).
   - Control data lives in internal RAM, never PSRAM.
   - NVS writes and OTA are refused unless the state is `Disabled`.
3. **Control task restrictions.** No blocking calls, mutexes, `malloc`, `printf` or `Serial` in the control task.
   - Out of the task: an SPSC ring buffer (telemetry) and a seqlock snapshot (latest state for console/web).
   - Into the task: a FreeRTOS queue polled non-blocking at the start of each cycle (settings changes, mode, references, test currents).
4. **Arduino `loop()`.** It runs on core 1. Leave it empty: `vTaskDelete(NULL)` in `setup()` after creating the tasks.
5. **Cycle budget** at 960 Hz is 1.04 ms. Target is under 300 µs from INT1 to the CAN frame being queued. Measure it in firmware in Stage 5: the INT1 ISR stores an `esp_timer` timestamp, the control task records the time when the `0x010` frame is queued, and the cycle time goes into the telemetry sample (§8.4). No timing pin.

### 8.3 Console (text, 115200 baud nominal; it's USB CDC)

| Command | From stage | Does |
|---|---|---|
| `ver` | 0 | Version banner |
| `imuid` | 1 | WHO_AM_I read |
| `canping` | 1 | Send a test frame on `0x7F0` |
| `canloop` | 1 | Self-test in loopback mode |
| `stream q` | 2 | SFLP quaternion for IMU-Visualiser quaternion mode, 50 Hz. Enable the SFLP game vector only while this runs; it's a Stage 2 axis check only |
| `stream e` | 3 | `roll;pitch;yaw` in degrees from our filter (λ, θ, ψ) for IMU-Visualiser Euler mode |
| `stream off` | 2 | Stop streaming |
| `bin on/off` | 5 | Binary telemetry |
| `drv` | 4 | Driver link states, flags, supply |
| `start` | 4 | Start FOC on both drivers |
| `cur <L> <R>` | 4 | Test currents, clamped to 0.5 A, auto-zero after 2 s without a repeat |
| `stop` | 4 | Disable, 0 A |
| `zero` | 3 | Store mounting offset so θ = 0 now. Refused unless `Disabled` |
| `get/set <key> [value]`, `save` | 5 | Plain key-value settings. `save` refused while balancing |
| `pose <deg>` | 9 | Mode R target pose, e.g. `pose 180` |
| `drive <speed> <turn>` | 8 | Speed and turn-rate references |

### 8.4 Binary telemetry

- **Frame:** `0xA5 0x5A`, u8 length, u8 type, payload, CRC-16/CCITT.
- **Type 1, control sample (one per cycle, about 64 bytes):** t_us, θ, θ̇, λ, ψ, ψ̇, φ, φ̇, mode, θ_ref and the other references, u_ff, i_L and i_R commanded, measured iq L/R, state, gate reason bits, θ_sflp, cycle time in µs.
- **Bandwidth:** about 61 kB/s at 960 Hz, fine for USB full speed. Add a decimation setting in case the host can't keep up.
- **Host side:** `tools/telemetry.py` writes CSV with a header that includes the firmware version and settings.

---

## 9. CAN Bench assets (tools/can-bench)

- **`robot.toml`:** extend the BLDC-Driver profile with the controller's test frame `0x7F0`, and later anything the user adds.
- **Sequence "Fake drivers":**
  1. Send heartbeats on `0x101`/`0x102` (state IDLE, 11.1 V) every 100 ms.
  2. Expect `[0x10]` on `0x021`/`0x022` after `start`.
  3. Reply `10 3F 80 00 00`.
  4. Expect current frames on `0x010` within 50 ms.
  5. Stop the heartbeats and expect the frames' currents to go to 0 within 400 ms.

  Result is PASS/FAIL, runnable with `--sequence "Fake drivers" --exit`.

---

## 10. Stages

Each stage: **Tasks**, **Tests** (who runs them, with what), **Acceptance** (all must hold). Stop at the end of each stage and report.

### Hardware needed at each stage

The user builds the protoboard before Stage 1. At the start of each stage, check that the hardware below exists. If it doesn't, stop and say what's missing.

| Stage | Hardware that must exist | Where | Power | CAN bus |
|---|---|---|---|---|
| 0 | Bare Zero only | Desk | USB | None |
| 1 | Whole protoboard: 5 V buck, Zero, IMU, SIT1051T/3, CAN connector, termination jumper, test points | Desk | Bench supply ≈12 V (current-limited) into the buck; USB for data | CANable only; both ends terminated (CAN Bench 120 Ω on, protoboard jumper closed) |
| 2 | Nothing new | Desk; IMU turned by hand | USB | Not used |
| 3 | Protoboard mounted rigidly on the robot in its final place | Robot; drivers unpowered | USB | Not used |
| 4A | Nothing new | Robot or desk, drivers not connected | As Stage 1 | CANable impersonates the drivers; termination as Stage 1 |
| 4B | CAN cables to both MiniFOCs, battery lead to the buck, battery switch or kill connector | Robot on a stand, wheels off the ground | Current-limited bench supply at 11–12 V first, then the battery | MiniFOCs at the two ends (jumpers closed); protoboard jumper open; CANable on a short stub, silent mode, termination off |
| 5–6 | Nothing new | Robot on the stand | Battery | As 4B |
| 7+ | Tether; USB lead for telemetry | Floor | Battery | As 4B |

USB alone gives the 5V pin only about 4.6–4.7 V (VBUS through a Schottky), only just above the SIT1051's 4.5 V minimum, with no margin for a weak USB port. Any test that uses CAN therefore needs the buck powered.

### Stage 0: Toolchain and bare board
- **Tasks:** §3 project, pinned platform, board JSON, banner, RGB LED cycle (RMT or the core's `rgbLedWrite`), BOOT logged, `native` env with one test.
- **Tests:** flash over USB; `pio test -e native`.
- **Acceptance:** the same commit builds identically on two machines; 10 of 10 USB flashes succeed.

### Stage 1: Protoboard bring-up (no motors)
- **Tasks:** `PinConfig.h`, SPI and WHO_AM_I, CAN init (§4.1), `imuid`, `canping`, `canloop`.
- **Tests:**
  - Meter on the 5 V and 3.3 V rails; scope on SCK and MISO at 10 MHz.
  - `canloop` passes.
  - CAN Bench on the bus (Normal mode, 1 Mbit/s, 87.5 %): `0x7F0` seen and ACKed every 100 ms; frames sent from CAN Bench are echoed.
- **Acceptance:** 10 min with 0 bus errors on both sides; 1000/1000 WHO_AM_I reads correct.

### Stage 2: IMU stream
- **Tasks:** §5 configuration and read path (control task skeleton driven by INT1, no CAN output yet), `stream q`, overrun counters.
- **Tests (IMU-Visualiser, quaternion mode):**
  - Rotate about each axis; find `R_sb` with the app's remap, then put it in firmware until no remap is needed.
  - Log rates: 960 accel+gyro samples/s, 960 cycles/s.
- **Acceptance:** axes right; sample rates within ±1 %; 0 overruns in 10 min.

### Stage 3: Our gravity filter
- **Tasks:** §6 filter and outputs, `stream e`, `zero`, unit tests §6.3, `r_imu` in config.
- **Tests:**
  - Angle jig or inclinometer at 0, ±10, ±45, ±90 and 180°.
  - **Swing test:** wheels held, body swung about the axle by hand. Compare θ against integrated gyro and against θ_sflp, with lever-arm compensation on and off.
  - **Slide test:** robot pushed along the table without tilting.
- **Acceptance:** jig error ≤ 0.5° everywhere including 180°; swing-test peak error with compensation ≤ 1° and below θ_sflp's; all §6.3 tests pass.

### Stage 4: MiniFOC link
- **Tasks:** §4.2–4.4, supervisor task, `drv`, `start`, `cur`, `stop`, velocity interval set to 1 ms.
- **Tests:**
  - (A) CAN Bench "Fake drivers" sequence.
  - (B) Real drivers, wheels off the ground, CAN Bench in silent mode with `bldc-driver.toml`: `cur 0.2 0.2` turns both wheels forward (fix with driver invert); plot command against velocity; unplug one driver and see the other stay up while the gate cuts current.
  - Measure bus load.
- **Acceptance:** (A) passes; signs match §4.4; bus load measured and recorded.

### Stage 5: Real-time loop at 0 A
- **Tasks:** full §8 task structure, safety gate §7.6, state machine stub (Disabled only), binary telemetry and `tools/telemetry.py`, INT1→CAN latency and cycle time measured in firmware (§8.2 rule 5).
- **Tests:**
  - Cycle time and INT1→CAN latency from telemetry (min, mean, max over 10 min).
  - CAN Bench gap monitor on `0x010` for 10 min, Wi-Fi off, then with a Wi-Fi STA connected and pinging (temporary test code on core 0).
  - Trigger an NVS write while running and confirm it's refused.
  - With motors spinning at a test current, log the gyro and report the spectrum: look for frame resonances.
- **Acceptance:** 0 missed cycles; worst `0x010` gap < 2 periods; INT1→CAN latency measured (target < 300 µs); gyro spectrum reported.

### Stage 6: Measure and design
- **Tasks:**
  - Spin test command (robot on a stand, current steps, logged), with a fit for K_t, friction and the speed filter lag.
  - Geometry (R, r_w, d_c, b, r_imu) and mass properties from CAD into `tools/model/robot.json`; compute θ_c.
  - Check ε (CoM offset from the ring centre): rest the robot on its rings at several poses and log whether it creeps and which way.
  - Wheel-mode model, LQR, feed-forward N_θ/N_u and margins (§7.2). Ring-mode model and gains (§7.3), with the wheel-speed drift estimate.
- **Tests:** the user reviews the measured parameters, θ_c and the margin report.
- **Acceptance:** margins met; `GainTable.inc` generated with the parameters recorded.

### Stage 7: First balance (mode W, upright)
- **Tasks:** mode detection (§7.1), mode W controller (§7.2, §7.5), references held still (v_cmd = 0), engage logic, I_max starting at 0.8 A.
- **Tests:** tethered first; then hands-off; pushes. Record each run.
- **Acceptance:** 5 min hands-off; recovers from a push; recordings saved.

### Stage 8: Driving and heading (mode W)
- **Tasks:** the reference generator with feed-forward (§7.4), `drive`, heading hold. The command source beyond serial is an open decision.
- **Tests:** 2 m out and back with set-point steps; spin on the spot; push while stopped; heading drift over 5 min. Plot θ against θ_ref during accelerations: the lean should lead the speed change, not lag it.
- **Acceptance:** returns within 3 cm after a push; heading drift < 2°/min; no overshoot beyond 10 % on a speed step.

### Stage 9: Ring mode, then extras
- **Tasks:** mode R controller (§7.3), `pose <deg>`. Then mode changes (wheels ↔ rings) once the trajectory is reviewed with the user. Then whatever the user picks from §11.
- **Acceptance:** holds 180° on its rings for 5 min and returns there after a nudge, with wheel speed staying under its limit. Mode changes: 10 of 10 in both directions.

## 11. Not in scope unless the user agrees

These exist in EBS-SBR. Don't implement them without a yes:

- Wi-Fi access point and web page, OTA
- BLE Xbox controller (note: NimBLE-Arduino 2.5.2 doesn't build on IDF 6.1)
- OLED display
- CAN modules protocol (`0x200 + m` drive requests, `0x180`/`0x181` robot state)
- Automatic motor supervisor with start retries
- Boot-time gyro calibration
- Learned set-point trim
- Balance priority at the current limit
- Lifted-wheel and runaway detection
- Automatic self-righting (rolling up from about 90° on the rings without being asked). The mode change in Stage 9 is the mechanism; automatic triggering is a separate decision
- Battery cutoff
- Settings registry with help text and import/export
- Boot timing log and device log
- Buzzer and button
- Reuse of EBS-SBR's Python tuning tools
- Driver configuration from the controller

The user will mark these; check the overview doc for the latest answers.

## 12. Open decisions (ask before the stage that needs them)

| Decision | Default | Needed before |
|---|---|---|
| IMU position `r_imu` relative to the axle | From CAD | Stage 3 |
| Geometry: R, r_w, d_c, b (θ_c follows) | From CAD | Stage 6 (placeholders before) |
| CoM offset ε from the ring centre | ≈ 0 by design; checked in Stage 6 | Stage 6 |
| Saturation priority | Plain per-wheel clamp | Stage 7 |
| Command source beyond serial | Serial only | Stage 8 |
| Second CAN controller | Unused; pins 47/49 kept free for an accessory bus | Before any CAN accessory |
| Battery monitor (INA226 on GPIO50/51) | Not on the protoboard; don't implement | When the user adds it |

## 13. References

- MiniFOC Mk2 board: https://github.com/clawntl/BLDC-Driver/blob/main/docs/boards/minifoc_mk2.md
- CAN protocol: https://github.com/clawntl/BLDC-Driver/blob/main/docs/can-protocol.md
- ACAN-ESP32FD API, filters and bit timing: https://github.com/clawntl/ACAN-ESP32FD/tree/main/docs
- ESP32-S31-Zero schematic and examples: https://github.com/waveshareteam/ESP32-S31-Zero
- ESP32-S31 pinout and IO guide: https://esp32-s31.espressif.com/en/docs/6-Pinout%20%26%20IO%20Introduction/pinout-and-io-allocation
- ESP32-S31 datasheet: https://documentation.espressif.com/esp32-s31_datasheet_en.html
- ST LSM6DSV16X driver: https://github.com/STMicroelectronics/lsm6dsv16x-pid
- ST SFLP note: https://community.st.com/t5/mems-and-sensors/how-lsm6dsv16x-enables-sensor-fusion-low-power-sflp-algorithm/ta-p/585084
- EBS-SBR (reference only): https://github.com/engbuildspace/EBS-SBR
