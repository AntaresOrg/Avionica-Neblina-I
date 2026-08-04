# Telemetry and Logging

## Flight Sample

The central data structure is `flight_sample_t` in `include/flight_log.h`.

It includes:

- timestamp in milliseconds
- BMP280 relative altitude for two sensors
- MPU6050 linear acceleration and gyro data for two sensors
- GPS latitude, longitude, altitude, and satellite count

## Flash Log

`src/flight_log.c` manages the persistent log stored in external SPI flash.

The log configuration in `src/main.cpp` uses:

- base address `0x000000`
- size `8 * 1024 * 1024`

The code supports:

- appending samples
- counting stored records
- reading records back
- dumping CSV to a writer callback

## Telemetry

`include/telemetry.h` defines two transmit modes:

- `TX_MODE_CSV`
- `TX_MODE_STRUCT`

The current configuration selects CSV mode through `kTelemetryTxMode` in `include/avionics_config.h`.

## Transmission Flow

In `src/main.cpp`:

- the firmware formats a CSV line for each sample
- it writes the line to serial output
- it appends the sample to flash when available
- it sends the line or a packed telemetry struct over LoRa only after a successful save

## Readback Mode

The firmware also supports flash readback mode, where the stored log is dumped instead of entering the normal flight loop.
