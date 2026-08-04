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

## Transition Logic

The implementation in `src/flight_state.cpp` is altitude based:

- It tracks the maximum observed altitude.
- It marks chute deployment when the altitude drops below the max by more than the configured offset.
- It marks reef deployment when the altitude falls below the reef altitude threshold after chute deployment.
- It moves to recovery when chute and reef are deployed and altitude falls below the ground threshold.

## Current Behavior

The state machine is used in `src/main.cpp` to decide whether the firmware should emit ground component status or normal flight telemetry.

What it does not currently do:

- it does not directly command the chute or reef outputs
- it does not implement explicit arm/safe states
- it does not use acceleration or velocity as launch detection inputs
- it does not expose a separate abort state

## Practical Interpretation

This is a phase classifier plus deployment-intent tracker, not yet a full mission sequencer.
