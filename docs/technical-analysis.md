# Technical Analysis

## What Is Already Implemented

- Two pressure sensors for redundancy.
- Two IMUs for inertial redundancy.
- GPS capture and sample enrichment.
- External flash logging with CSV dump support.
- LoRa telemetry output.
- A phase-based flight controller.
- Output drivers for chute and reef charges.

## What Is Still Missing For a Full Flight Plan

- explicit arming and safing logic
- launch detection based on acceleration or velocity, not only altitude
- explicit apogee detection and deployment decision logic
- direct charge actuation tied to the flight controller
- abort handling and fault-state handling
- sensor sanity checks before deployment
- post-landing confirmation and recovery completion criteria
- a formal preflight / in-flight / postflight mode separation

## Current Risk Areas

### 1. State Machine vs Actuation Gap

The code computes `chute_deployed` and `reef_deployed`, but no code path in `src/main.cpp` calls `charges_set_chute()` or `charges_set_reef()`.

### 2. Ground/Flight Interface Is Implicit

The system decides when to send ground-status telemetry, but there is no explicit operator command channel shown in the current firmware.

### 3. Threshold-Only Flight Logic

Altitude thresholds are sufficient for a simple demo, but a flight computer usually needs inertial cues and a more explicit event chain.

### 4. Configuration Drift

The root `README.md` still describes a different boot mode state than the current `include/avionics_config.h` values.

## Recommended Next Steps

1. Add a formal arm/safe state and a deployment guard.
2. Connect the flight phases to charge outputs with explicit timing.
3. Add launch and apogee detection rules using the MPU6050 and BMP280 together.
4. Add fault handling for invalid sensors and storage failure.
5. Add a short mission checklist page for ground operations and launch day procedures.
