#include <stdio.h>
#include <math.h>
#include "lora.h"
#include "flash_memory.h"
#include "flight_log.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "gps.h"
#include "telemetry.h"
#include "flight_state.h"
#include "avionics_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

static const char *TAG = "SENSORS";

static bool lora_ready = false;
static bool gps_ready = false;
static bool flash_ready = false;
static flight_log_t flight_log;
static flight_state_controller_t flight_controller;
static uint32_t last_lora_tx_ms = 0;
static bool telemetry_header_sent = false;

static void log_line_serial_only(const char *line);

static void write_line_serial_and_lora(void *ctx, const char *line)
{
    (void)ctx;
    log_line_serial_only(line);
    if (lora_ready)
        lora_send_line(line);
}

static void dump_flight_log(void)
{
    if (!flash_ready || !flight_log.initialized)
    {
        ESP_LOGW(TAG, "Flash/log not ready; cannot read back");
        return;
    }

    esp_err_t err = flight_log_dump_csv(&flight_log, write_line_serial_and_lora, NULL, true);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Flight log dump failed: %s", esp_err_to_name(err));
}

static void poll_lora_rx(void)
{
    if (!lora_ready)
        return;

    // Drain and print everything we receive from LoRa UART.
    // NOTE: This prints to UART0 via printf() and can interleave with CSV output.
    while (lora_available())
    {
        char buf[128];
        int n = lora_read(buf, sizeof(buf));
        if (n <= 0)
            break;

        printf("LORA_RX_RAW: %.*s\n", n, buf);
    }
}

static void log_line_serial_only(const char *line)
{
    if (!line)
        return;

    // Plain UART0 output (serial monitor friendly; no ESP_LOG prefixes).
    printf("%s\n", line);
}

static avionics_component_status_t get_component_status(void)
{
    avionics_component_status_t status = {
        .lora_ready = lora_ready,
        .gps_ready = gps_ready,
        .flash_ready = flash_ready,
        .bmp1_ready = bmp280_is_initialized(0),
        .bmp2_ready = bmp280_is_initialized(1),
        .mpu1_ready = mpu6050_is_initialized(0),
        .mpu2_ready = mpu6050_is_initialized(1),
    };

    return status;
}

static void send_ground_component_status(void)
{
    if (!lora_ready)
        return;

    char line[192];
    avionics_component_status_t status = get_component_status();
    if (flight_state_format_component_status(&flight_controller, &status, line, sizeof(line)) == ESP_OK)
        lora_send_line(line);
}

/**
 * @brief Configure and start ESP32 I2C master peripheral.
 *
 * Uses the constants defined at the top of this file for pins, port and bus speed.
 */
void i2c_master_init(void)
{
    i2c_config_t conf = {};

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = kI2cMasterSdaIo;
    conf.scl_io_num = kI2cMasterSclIo;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = kI2cMasterFreqHz;

    i2c_param_config(kI2cMasterNum, &conf);
    i2c_driver_install(kI2cMasterNum, conf.mode, 0, 0, 0);
}

// ---------- MAIN ----------
/**
 * @brief ESP-IDF application entry point.
 *
 * Initializes I2C and all sensors, then continuously reads and logs data.
 */
extern "C" void app_main(void)
{
    // Make sure CSV / line output appears immediately on UART0.
    setvbuf(stdout, NULL, _IONBF, 0);

    i2c_master_init();

    lora_ready = lora_init();
    if (lora_ready)
        ESP_LOGI(TAG, "LoRa initialized");
    else
        ESP_LOGW(TAG, "LoRa init failed, serial-only mode");

    const gps_config_t gps_cfg = kGpsConfig;

    esp_err_t gps_err = gps_init(&gps_cfg);
    gps_ready = (gps_err == ESP_OK);
    if (gps_ready)
        ESP_LOGI(TAG, "GPS initialized");
    else
        ESP_LOGW(TAG, "GPS init failed: %s", esp_err_to_name(gps_err));

    const flight_state_thresholds_t flight_thresholds = {
        .target_altitude_m = kTargetAltitudeM,
        .reef_altitude_m = kReefAltitudeM,
        .ground_altitude_m = kGroundAltitudeM,
    };
    flight_state_controller_init(&flight_controller, &flight_thresholds);

    const flash_memory_config_t flash_cfg = kFlashConfig;

    esp_err_t flash_err = flash_memory_init(&flash_cfg);
    flash_ready = (flash_err == ESP_OK);
    if (flash_ready)
        ESP_LOGI(TAG, "External SPI flash initialized");
    else
        ESP_LOGW(TAG, "External SPI flash init failed: %s", esp_err_to_name(flash_err));

    if (flash_ready)
    {
        flight_log_config_t log_cfg = {
            .base_address = 0x000000,
            .size_bytes = 8 * 1024 * 1024,
        };

        esp_err_t log_err = flight_log_init(&flight_log, &log_cfg);
        if (log_err == ESP_OK)
            ESP_LOGI(TAG, "Flight log ready (%u records)", (unsigned int)flight_log_count(&flight_log));
        else
            ESP_LOGW(TAG, "Flight log init failed: %s", esp_err_to_name(log_err));
    }

    if (kFlashReadbackMode)
    {
        ESP_LOGI(TAG, "READBACK MODE: kFlashReadbackMode=1");
        dump_flight_log();
        while (1)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (!kFlashReadbackMode && kResetFlightLogBeforeLogging && flash_ready && flight_log.initialized)
    {
        // Full erase of a multi-megabyte region at boot can take a long time and
        // trip the task watchdog. Instead, reset the write pointer and erase the
        // first sector now; remaining sectors will be erased on-demand as logging
        // reaches them.
        ESP_LOGW(TAG, "Resetting flight log before logging (kResetFlightLogBeforeLogging=1)");

        flight_log.write_offset = 0;

        esp_err_t erase_err = flash_memory_erase_4k(flight_log.cfg.base_address);
        if (erase_err != ESP_OK)
            ESP_LOGW(TAG, "Flight log first-sector erase failed: %s", esp_err_to_name(erase_err));
        else
            ESP_LOGI(TAG, "Flight log reset (starting from record 0)");
    }

    mpu6050_init_all();
    bmp280_init_all();

    while (1)
    {
        poll_lora_rx();
        gps_loop();
        mpu6050_sample_t m1;
        mpu6050_sample_t m2;
        float b1_alt = NAN;
        float b2_alt = NAN;

        mpu6050_read_all(&m1, &m2);
        bmp280_read_all(&b1_alt, &b2_alt);

        // Always build a timestamped sample, even if sensors failed.
        // Missing values are stored/logged as NAN.
        flight_sample_t s = {
        .time_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS),

        .bmp1_relative_altitude_m = b1_alt,
        .bmp2_relative_altitude_m = b2_alt,

        .mpu1_ax_g = m1.valid ? m1.ax_g : NAN,
        .mpu1_ay_g = m1.valid ? m1.ay_g : NAN,
        .mpu1_az_g = m1.valid ? m1.az_g : NAN,
        .mpu1_gx_dps = m1.valid ? m1.gx_dps : NAN,
        .mpu1_gy_dps = m1.valid ? m1.gy_dps : NAN,
        .mpu1_gz_dps = m1.valid ? m1.gz_dps : NAN,

        .mpu2_ax_g = m2.valid ? m2.ax_g : NAN,
        .mpu2_ay_g = m2.valid ? m2.ay_g : NAN,
        .mpu2_az_g = m2.valid ? m2.az_g : NAN,
        .mpu2_gx_dps = m2.valid ? m2.gx_dps : NAN,
        .mpu2_gy_dps = m2.valid ? m2.gy_dps : NAN,
        .mpu2_gz_dps = m2.valid ? m2.gz_dps : NAN,

        .gps_lat_deg = NAN,
        .gps_lon_deg = NAN,
        .gps_altitude_m = NAN,
        .gps_satellites = NAN,
    };

        gps_fill_sample(&s, gps_cfg.max_fix_age_ms);

        float current_altitude_m = flight_state_select_current_altitude(
            s.bmp1_relative_altitude_m,
            s.bmp2_relative_altitude_m);
        flight_state_phase_t previous_phase = flight_controller.phase;
        flight_state_phase_t current_phase = flight_state_controller_update(&flight_controller, current_altitude_m);

        if (current_phase != previous_phase)
        {
            ESP_LOGI(TAG,
                     "Flight state -> %s (alt=%.2f m, reef=%u, chute=%u)",
                     flight_state_name(current_phase),
                     flight_controller.current_altitude_m,
                     flight_controller.reef_deployed ? 1u : 0u,
                     flight_controller.chute_deployed ? 1u : 0u);
        }

        if (flight_state_should_send_component_status(&flight_controller))
        {
            uint32_t now_ms = s.time_ms;
            if (last_lora_tx_ms == 0 || (uint32_t)(now_ms - last_lora_tx_ms) >= kLoraTxIntervalMs)
            {
                send_ground_component_status();
                last_lora_tx_ms = now_ms;
            }

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (!telemetry_header_sent)
        {
            char header[384];
            if (flight_log_format_csv_header(header, sizeof(header)) == ESP_OK)
            {
                log_line_serial_only(header);
                telemetry_header_sent = true;
            }
        }

        // Always emit the timestamped line to serial.
        // LoRa should transmit only packets that were successfully saved to flash.
        char line[384];
        bool line_ok = (telemetry_format_sample_csv(&s, line, sizeof(line)) == ESP_OK);
        if (line_ok)
            log_line_serial_only(line);

        bool saved = false;
        if (flash_ready && flight_log.initialized)
        {
            esp_err_t append_err = flight_log_append(&flight_log, &s);
            if (append_err != ESP_OK)
            {
                ESP_LOGW(TAG, "Flight log append failed: %s", esp_err_to_name(append_err));
            }
            else
            {
                saved = true;
            }
        }

        if (saved && lora_ready && line_ok)
        {
            uint32_t now_ms = s.time_ms;
            if (last_lora_tx_ms == 0 || (uint32_t)(now_ms - last_lora_tx_ms) >= kLoraTxIntervalMs)
            {
                if (kTelemetryTxMode == TX_MODE_STRUCT)
                {
                    telemetry_packet_t packet = telemetry_build_packet(&s);
                    lora_send_bytes(&packet, sizeof(packet));
                }
                else
                {
                    lora_send_line(line);
                }

                last_lora_tx_ms = now_ms;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}