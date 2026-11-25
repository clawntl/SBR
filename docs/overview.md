# ESP32-S31 balancing robot controller — implementation plan

Nov 25, 2026 · @Someone

## Summary

The controller is built in 10 stages, each ending in a test with CAN Bench, IMU-Visualiser or the serial console. Motors first turn in Stage 4 with the wheels off the ground; the robot first balances in Stage 7. This doc is the overview; the detailed spec for Claude Code is a separate file, `docs/implementation-plan.md`.

### Decided

| Topic | Decision |
| --- | --- |
| Framework | Arduino 4.0 on PlatformIO (pioarduino), with ESP-IDF APIs where timing matters |
| Robot | Two rings in the pitch plane, centred close to the centre of mass, well above the wheel axle. On its wheels it is an inverted pendulum; on its rings gravity pulls only weakly, through the small offset between the centre of mass and the ring centre |
| Hardware | One protoboard near the top of the body: the Zero, the IMU, a CAN transceiver and a 5 V buck. 3S 18650 Li-ion battery |
| CAN library | [ACAN-ESP32FD](https://github.com/clawntl/ACAN-ESP32FD): classic CAN, 1 Mbit/s, 87.5 % sample point |
| IMU interrupt | INT1 only: FIFO watermark into GPIO46 |
| Loop rate | 960 Hz: one control cycle per IMU sample |
| Attitude | Our own gravity-vector filter on raw gyro and accel, correcting for the IMU's height; the IMU's on-chip fusion (SFLP) gravity output only as a cross-check |
| Controller | One 6-state LQR (wheel position, tilt, heading and their rates), gains scheduled by body angle so it can balance upright or inverted |
| Battery sense | Later, with an INA226 on I²C; not on the protoboard |
| EBS-SBR features | Only by agreement (section 5) |

### Still open

| Decision | My suggestion | Needed before |
| --- | --- | --- |
| Which tilt angles have the wheels on the ground, and which only the ring | Send me the ring radius, wheel radius and axle-to-ring-centre distance, and I'll work it out | Stage 6 |
| IMU offset from the axle | Measured from CAD | Stage 3 |
| Second CAN controller | Leave it free now; later, an accessory bus so the motor bus stays private | Before adding CAN accessories |
| Driving and tuning beyond USB serial | Serial until Stage 8; web page later, on core 0 | Stage 8 |

## Hardware at a glance

The Zero, the IMU, a CAN transceiver and a 5 V buck share one protoboard near the top of the body; 7 of its 15 header pins are used.

&#91;embedded content: Zero header pin map · 7 used, 8 spare\]

- **Power:** 3S 18650 pack to a 5 V buck (1 A) on the protoboard, into the Zero's 5V pin. A Schottky on the Zero makes this safe with USB plugged in too. The Zero's 3V3 feeds the IMU and the transceiver's logic side.
- **CAN:** a SIT1051T/3 (the version with a VIO pin): VCC 5 V, VIO 3.3 V so its RXD output never exceeds 3.3 V, S tied low for normal mode. One JST-GH 3-pin connector per driver. Leave the termination jumper open if the drivers are at the bus ends.
- **IMU:** on the protoboard near the top of the body, mounted rigidly with axes x forward, y along the axle, z up, on SPI at its 10 MHz maximum. Its offsets from the axle and from the ring centre go in the config, so the filter can subtract the extra acceleration it feels (see Risks).
- **Test points:** GND, 3V3, CANH/CANL, INT1 and GPIO48.

## How it works

One task on core 1 runs each control cycle when the IMU's INT1 fires; Wi-Fi, the web page and everything else stay on core 0.

&#91;embedded content: control cycle and tasks · 2 cores\]

### The controller

One LQR on all six states, heading and turn rate included, sets a common current and a differential current; what is touching the ground (wheels or rings) and the pose pick which gains it uses.

&#91;embedded content: controller structure · 6 states, 2 inputs\]

- **Why one LQR covers yaw too:** for a left-right symmetric robot, position and tilt only respond to the common current, and heading only to the difference. A design on the full model comes out with that block structure anyway, and it still handles any coupling the model has.
- **Upside down:** tilt comes from the gravity direction, so it is valid all the way round. Upside down the wheels are in the air and the robot rests on its rings, so holding 180° is a job for ring mode (next point), not the wheel-mode LQR.
- **Heading** integrates the turn rate about the world vertical (gyro · gravity), so it stays right when the body is tilted or inverted.
- **Two contact modes:** Wheel mode, near upright with the wheels on the ground: an inverted pendulum, because the centre of mass sits above the axle. The 6-state LQR runs here. Ring mode, every other pose including upside down: the rings carry the robot. With the centre of mass close to their centre, gravity pulls only weakly: the robot settles where the centre of mass is lowest, and other poses need a small holding torque. The wheels are in the air and work as reaction wheels; turning them rolls the robot along its rings. Its position is tied to its pose, and it can't steer. Ring mode needs its own small controller, which also stops the wheels running away in speed. The angle where the modes swap comes from the geometry (still open).
- **Driving uses set points:** a speed or turn command never goes in as a current. It sets the controller's references: wheel speed and heading rate, ramped to an acceleration limit; wheel position and heading, integrated from them; and the lean the robot needs to accelerate, plus a matching feed-forward current. Both come from the model. The LQR then holds the robot on that moving reference, so the robot is never pushed into falling forward.

### Keeping a web page out of the control path

- Wi-Fi, the web server and OTA run on core 0, and Wi-Fi is started from a core-0 task. The IMU, SPI and CAN interrupts are set up from core 1.
- Writing flash stalls both cores for tens of milliseconds. So the control code runs from RAM, settings are only saved when not balancing, and OTA only runs with the motors off.
- The control task never waits on a lock: it publishes snapshots, and the web side reads them.
- Arduino's `loop()` runs on core 1, so it stays empty.

## Roadmap

The motors stay off through Stage 3, turn only with the wheels lifted in Stages 4–5, and the robot first balances in Stage 7.

&#91;embedded content: roadmap · 10 stages in 3 phases\]

Each stage's tasks, tests and acceptance numbers are spelled out in the Claude Code spec.

### Hardware needed at each stage

Build the whole protoboard before Stage 1, mount it on the robot at the start of Stage 3, and connect it to the drivers and the battery at Stage 4.

| Stage | What has to exist | Where | Power | CAN bus |
| --- | --- | --- | --- | --- |
| 0 | Nothing: just the bare Zero | Desk | USB | None |
| 1 | The whole protoboard: 5 V buck, Zero, IMU, SIT1051, a CAN connector, termination jumper, test points | Desk | Bench supply at about 12 V, current-limited, into the buck; USB for data | CANable only. Terminate both ends: CAN Bench's 120 Ω on, protoboard jumper closed |
| 2 | Nothing new | Desk; IMU turned by hand | USB is enough | Not used |
| 3 | Mount the protoboard on the robot, rigidly, in its final place | Robot; drivers unpowered | USB | Not used |
| 4 (test A) | Nothing new | Robot or desk, drivers not connected | As Stage 1 | CANable plays the drivers; same termination as Stage 1 |
| 4 (test B) | CAN cables to both MiniFOCs; battery lead to the buck; a battery switch or kill connector | Robot on a stand, wheels off the ground | Bench supply at 11–12 V with a current limit for the first runs, then the battery | MiniFOCs at the two ends with their jumpers closed; protoboard jumper open; CANable on a short stub, silent mode, its termination off |
| 5–6 | Nothing new | Robot on the stand | Battery | As Stage 4B |
| 7 onwards | A tether (and a USB lead for telemetry) | Floor | Battery | As Stage 4B |

USB alone can't run the transceiver reliably: the Zero's 5V pin sits a diode drop below USB's 5 V, about 4.6–4.7 V, against the SIT1051's 4.5 V minimum. So any test that uses CAN powers the buck.

## EBS-SBR features: yours to choose

None of these are in the plan yet. Mark each one yes, no or later and I'll fold the yeses into Stage 9 (or earlier where noted). Two things in the plan overlap with EBS-SBR but are standard practice, so I kept them unless you object: LQR state feedback with gains from a model, and a wheel spin test to measure the torque constant.

| Feature in EBS-SBR | What it does there | Notes for this robot |
| --- | --- | --- |
| Wi-Fi access point and web page | Tune, Control, WiFi, Log and Info tabs over a WebSocket; live charts | Biggest single piece of work. Cheaper option: keep USB serial plus a host script |
| OTA updates | Flash over Wi-Fi | Only useful with Wi-Fi |
| BLE Xbox controller | Drive with a gamepad | NimBLE-Arduino 2.5.2 doesn't build on IDF 6.1, per EBS-SBR's README: check before choosing |
| OLED display | Boot steps, state, tilt, currents, faults | Needs I²C on GPIO50/51 |
| CAN modules protocol | Other boards drive the robot (`0x200 + m`); robot broadcasts state (`0x180`, `0x181`) | Would also make the robot's state visible in CAN Bench. Uses filter slots on the S31 (3 mask + 1 range) |
| Motor supervisor | Starts drivers one at a time, retries, restarts a driver that reset | Stage 4 only has a manual start; some automatic start is needed before Stage 7 anyway |
| Boot-time gyro calibration | Saved offsets, re-zero, checks the robot is still | Our filter tracks gyro bias all the time; a still-at-boot check is cheap if you want one |
| Learned set-point trim | Adjusts the balance angle while running | Little use with the CoM at the ring centre |
| Balance priority at the current limit | Drops position and speed terms when current saturates | Worth considering once Stage 7 shows how often it saturates |
| Lifted-wheel and runaway detection | Stops when a wheel spins free | A runaway cut-out is cheap safety; I'd suggest at least that before Stage 7 |
| Side cut-out | Stops balancing when lying on its side | Overlaps the side-lean limit already in Stage 5 |
| Self-righting | Rocks itself up off its hoops using the wheels as reaction wheels | Needed later: rolling up on the rings from lying at 90° onto the wheels. That's ring mode plus the hand-over to wheel mode (Stage 9), so the wheels as reaction wheels can do it |
| Battery cutoff | Stops below 9.6 V using the drivers' supply readings | Strongly suggested with Li-ion |
| Recording to CSV | Every control step, downloadable, with settings in the header | Stage 5's USB stream covers the basics |
| Settings registry | Named settings with ranges and help text, NVS, import and export | Stage 7 only needs a handful of values |
| Boot timing log, device log | Times each boot step; recent messages on the web page |  |
| Status LED, button, buzzer | State colours and sounds | LED and BOOT button are on the Zero already |
| Python tuning tools | `sbr_model.py`, `design_gains.py`, `fit_spin.py`, `check_log.py`, `self_right.py` | Stage 6 needs a model and gain design: reuse these, adapt them, or write new ones for CoM at the axle |
| Driver configuration from the controller | Node ID, invert, current loop, anti-cogging over CAN | CAN Bench's Settings window already does this |

## Risks

The two biggest risks both come from the IMU sitting high on the body: extra acceleration and frame flex. Stage 3 and Stage 5 measure them before anything balances.

| Risk | Why | What we do |
| --- | --- | --- |
| IMU far from the axle | Up the body, the accelerometer also feels the body's angular acceleration and centripetal acceleration, as well as the wheels' acceleration. A hard recovery could otherwise read as several degrees of tilt | The filter subtracts all three using the measured offset, the gyro and the wheel speeds. It trusts the accelerometer less while that predicted acceleration is large. Stage 3's swing test checks it |
| Frame flex and vibration | The top of the body moves most; flex appears in the gyro as a resonance the controller can excite | Stiff mount. Log the gyro spectrum with motors running in Stage 5; filter the tilt rate if a peak shows |
| Flash writes stall both cores | Tens of milliseconds; the drivers hold the last current for up to 100 ms | Control code in RAM, no saves while balancing, OTA only with motors off |
| S31 is still a preview target | Toolchain and driver bugs, few examples | Stage 0 proves the toolchain before the protoboard is wired; every version pinned |
| Small CoM offset from the ring centre | A few mm either way turns neutral into slightly unstable or slightly stable | Stage 6 measures it and designs for both signs |
| Wi-Fi jitter on the control core | New chip, new radio | Stage 5 timing test with Wi-Fi off, then on |

## Sources

- [EBS-SBR (engbuildspace)](https://github.com/engbuildspace/EBS-SBR) and [EBS-SBR (clawntl)](https://github.com/clawntl/EBS-SBR)
- [BLDC-Driver: MiniFOC Mk2 board page](https://github.com/clawntl/BLDC-Driver/blob/main/docs/boards/minifoc_mk2.md) and [CAN protocol](https://github.com/clawntl/BLDC-Driver/blob/main/docs/can-protocol.md)
- [CAN Bench](https://github.com/clawntl/can-bench) and its [device profile reference](https://github.com/clawntl/can-bench/blob/main/profiles/README.md)
- [IMU-Visualiser](https://github.com/clawntl/IMU-Visualiser)
- [ESP32-S31-Zero docs](https://docs.waveshare.com/ESP32-S31-Zero), [repository and schematic](https://github.com/waveshareteam/ESP32-S31-Zero)
- [ESP32-S31 pinout and IO guide](https://esp32-s31.espressif.com/en/docs/6-Pinout%20%26%20IO%20Introduction/pinout-and-io-allocation), [ESP32-S31 datasheet](https://documentation.espressif.com/esp32-s31_datasheet_en.html)
- [ESP-IDF](https://github.com/espressif/esp-idf) (`components/soc/esp32s31` capabilities), [Arduino-ESP32](https://github.com/espressif/arduino-esp32) 4.0.0 board list
- [ACAN-ESP32FD](https://github.com/clawntl/ACAN-ESP32FD)
- [ST lsm6dsv16x-pid driver](https://github.com/STMicroelectronics/lsm6dsv16x-pid), [ST: how LSM6DSV16X enables SFLP](https://community.st.com/t5/mems-and-sensors/how-lsm6dsv16x-enables-sensor-fusion-low-power-sflp-algorithm/ta-p/585084), [ST: half-precision error near 180°](https://community.st.com/mems-sensors-48/lsm6dsv-half-precision-error-at-180-degrees-126519), [ST: gyroscope bias](https://community.st.com/t5/mems-sensors/lsm6dsv16x-gyroscope-bias/td-p/714927)
