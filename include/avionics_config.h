#pragma once

#include <stdint.h>

#include "flash_memory.h"
#include "driver/i2c.h"
#include "gps.h"
#include "telemetry.h"

static constexpr i2c_port_t kI2cMasterNum = I2C_NUM_0;
static constexpr gpio_num_t kI2cMasterSclIo = GPIO_NUM_22;
static constexpr gpio_num_t kI2cMasterSdaIo = GPIO_NUM_21;
static constexpr int kI2cMasterFreqHz = 100000;

static constexpr bool kFlashReadbackMode = false;
static constexpr bool kFlashResetBeforeReadback = false;
static constexpr uint32_t kLoraTxIntervalMs = 1000u;

static constexpr telemetry_tx_mode_t kTelemetryTxMode = TX_MODE_CSV;

static constexpr gps_config_t kGpsConfig = {
    .uart_num = UART_NUM_1,
    .baud_rate = 9600,
    .tx_pin = GPIO_NUM_27,
    .rx_pin = GPIO_NUM_14,
    .rx_buffer_size = 2048,
    .max_fix_age_ms = 2000,
};

static constexpr flash_memory_config_t kFlashConfig = {
    .host = SPI3_HOST,
    .mosi_io_num = GPIO_NUM_23,
    .miso_io_num = GPIO_NUM_19,
    .sclk_io_num = GPIO_NUM_18,
    .cs_io_num = GPIO_NUM_5,
    .clock_speed_hz = 8000000,
};