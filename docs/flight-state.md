# Flight State Machine

## Phases

The flight controller exposes these phases in `include/flight_state.h`:

- `GROUND`
- `FLIGHT`
- `CHUTE`
- `REEF`
- `RECOVERY`

## Controller Inputs

The controller is seeded from the thresholds in `include/avionics_config.h`:

- `kAltitudeOffsetM`
- `kReefAltitudeM`
- `kGroundAltitudeM`
- `kDebugFlightModeOnly`

It also tracks mission flags:

- `armed`
- `faulted`
- `launch_detected`
- `chute_commanded`
- `reef_commanded`

## Transition Logic

The implementation in `src/flight_state.cpp` is altitude based after the flight is armed and a launch is detected:

- It tracks the maximum observed altitude.
- It marks chute deployment when the altitude drops below the max by more than the configured offset.
- It marks reef deployment when the altitude falls below the reef altitude threshold after chute deployment.
- It moves to recovery when chute and reef are deployed and altitude falls below the ground threshold.

The controller is held in `GROUND` until all of the following are true:

- the controller is armed
- no fault is set
- launch has been detected

That means the nominal flight sequence is now:

1. `GROUND` for preflight and ground-status mode.
2. `FLIGHT` after launch has been detected and the controller begins tracking altitude.
3. `CHUTE` once descent exceeds the configured altitude offset from the peak.
4. `REEF` after chute deployment and descent below the reef threshold.
5. `RECOVERY` after both deployment events and return below the ground threshold.

## Current Behavior

The state machine is used in `src/main.cpp` to:

- switch between ground-status telemetry and normal flight telemetry
- trigger the chute charge once on the `CHUTE` transition
- trigger the reef charge once on the `REEF` transition
- prevent arming from surviving a fault
- keep the outputs off when the controller is not armed or is faulted

Launch detection in the main loop is driven by the MPU6050 acceleration magnitude check, while altitude comes from the two BMP280 sensors.

## Safety Behavior

The controller now has a simple safety model:

- `flight_state_controller_set_armed()` enables deployment only when the controller is not faulted.
- `flight_state_controller_set_faulted()` clears arming immediately.
- `flight_state_controller_should_fire_chute()` and `flight_state_controller_should_fire_reef()` are one-shot guards.
- `charges_all_off()` is used at boot to force both outputs low before any flight action.

## Practical Interpretation

This is a compact mission sequencer for a two-stage deployment profile with telemetry, logging, and fault gating.
