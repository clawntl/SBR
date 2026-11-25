# CLAUDE.md

Controller firmware for a two-wheeled balancing robot with rings (sister project to EBS-SBR): Waveshare ESP32-S31-Zero, LSM6DSV16X IMU, two MiniFOC Mk2 drivers on CAN, Arduino 4.0 on PlatformIO.

**Read `docs/implementation-plan.md` before doing anything.** It is the spec: pins, protocols, estimator, controller, task rules, and the staged plan with tests and acceptance checks.

## Rules

- Work one stage at a time (spec §10). At the start of a stage, check the hardware for it exists (spec §10, "Hardware needed at each stage"). At the end, run the stage's tests, report against its acceptance checks, and stop for the user to confirm.
- Don't add features the spec doesn't ask for, and don't copy EBS-SBR features (spec §11) without asking.
- Ask before changing pins, CAN IDs, the protocol, dependencies, or anything listed as decided (spec §1.3). If a stage needs an open decision (spec §12), ask.
- Every motor current goes through the safety gate. Never write code that can drive the motors outside the stage that introduces it.
- Pin every version (platform, libraries, vendored code).

## Current stage

Stage 0 (not started).

Update this line when a stage is accepted.
