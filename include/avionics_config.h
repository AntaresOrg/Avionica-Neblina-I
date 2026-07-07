#pragma once

#include <stdint.h>

#include "charges.h"
#include "flash_memory.h"
#include "driver/i2c.h"
#include "gps.h"
#include "telemetry.h"

// Shared hardware and mode configuration used by the firmware.
// Keep this file in sync with the physical wiring and boot mode of the avionics.

// Main I2C bus used by the BMP280 and MPU6050 sensors.
static constexpr i2c_port_t kI2cMasterNum = I2C_NUM_0;
// I2C SCL line on the ESP32.
static constexpr gpio_num_t kI2cMasterSclIo = GPIO_NUM_22;
// I2C SDA line on the ESP32.
static constexpr gpio_num_t kI2cMasterSdaIo = GPIO_NUM_21;
// Standard 100 kHz I2C bus speed.
static constexpr int kI2cMasterFreqHz = 100000;

// When true, the firmware boots in flash readback mode instead of normal logging.
static constexpr bool kFlashReadbackMode = false;
// When booting for normal logging, reset the flight log before appending new data.
static constexpr bool kResetFlightLogBeforeLogging = false;
// Period between LoRa telemetry transmissions in milliseconds.
static constexpr uint32_t kLoraTxIntervalMs = 1000u;
// When true, keep the avionics in FLIGHT mode only for debug and ignore altitude-based transitions.
static constexpr bool kDebugFlightModeOnly = false;

// Flight-state altitude thresholds in meters, based on BMP280 relative altitude.
static constexpr float kAltitudeOffsetM = 10.0f;
static constexpr float kReefAltitudeM = 150.0f;
static constexpr float kGroundAltitudeM = 5.0f;

// Telemetry packet format sent over LoRa.
static constexpr telemetry_tx_mode_t kTelemetryTxMode = TX_MODE_CSV;

// GPS/Neo-6M UART wiring and parser settings.
// UART1 TX is kept for completeness; the important receive path is GPS TX -> ESP RX.
static constexpr gps_config_t kGpsConfig = {
    .uart_num = UART_NUM_1,
    .baud_rate = 9600,
    .tx_pin = GPIO_NUM_27,
    .rx_pin = GPIO_NUM_14,
    .rx_buffer_size = 2048,
    .max_fix_age_ms = 2000,
};

// External SPI flash wiring for the W25Q128 chip.
static constexpr flash_memory_config_t kFlashConfig = {
    .host = SPI3_HOST,
    .mosi_io_num = GPIO_NUM_23,
    .miso_io_num = GPIO_NUM_19,
    .sclk_io_num = GPIO_NUM_18,
    .cs_io_num = GPIO_NUM_5,
    .clock_speed_hz = 8000000,
};

// Output GPIOs for the chute and reef charges.
static constexpr charges_config_t kChargesConfig = {
    .reef_gpio = GPIO_NUM_26,
    .chute_gpio = GPIO_NUM_25,
};