#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include "lora.h"
#include "flash_memory.h"
#include "flight_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "neo-6m.h"
#include "mpu6050.h"
#include "bmp.h"
#include "i2c.h"

static const char *TAG = "SENSORS";
// Set to 1 to boot into flash readback (dump CSV), 0 for normal logging.
#define FLASH_READBACK_MODE 0
// Set to 1 to erase the flight log region on boot BEFORE logging starts.
// This runs only in normal logging mode (FLASH_READBACK_MODE=0).
#define FLASH_RESET_BEFORE_READBACK 1


static bool lora_ready = false;
static bool flash_ready = false;
static flight_log_t flight_log;

static void log_line_serial_only(const char *line)
{
    if (!line)
        return;

    // Plain UART0 output (serial monitor friendly; no ESP_LOG prefixes).
    printf("%s\n", line);
}

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

void publish_line(const char *fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    ESP_LOGI(TAG, "%s", line);
    if (lora_ready)
    {
        lora_send_line(line);
    }
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

    flash_memory_config_t flash_cfg = {
        .host = SPI3_HOST,
        .mosi_io_num = GPIO_NUM_23,
        .miso_io_num = GPIO_NUM_19,
        .sclk_io_num = GPIO_NUM_18,
        .cs_io_num = GPIO_NUM_5,
        .clock_speed_hz = 8000000,
    };

    neo6m_config_t gps_cfg = {
        .uart_num = UART_NUM_1,
        .baud_rate = 9600,
        // Neo-6M UART1 wiring (module header: VCC RX TX GND):
        // - GPS TX -> ESP RX (GPIO14)
        // - GPS RX -> ESP TX (GPIO27)
        // - GPS VCC -> 5V/VIN, GPS GND -> GND
        .tx_pin = GPIO_NUM_27,
        .rx_pin = GPIO_NUM_14,
        .rx_buffer_size = 2048,
        .max_fix_age_ms = 2000,
    };

    ESP_ERROR_CHECK(neo6m_setup(&gps_cfg));

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

    if (FLASH_READBACK_MODE)
    {
        ESP_LOGI(TAG, "READBACK MODE: FLASH_READBACK_MODE=1");
        dump_flight_log();
        while (1)
            vTaskDelay(pdMS_TO_TICKS(1000));
    }

    if (FLASH_RESET_BEFORE_READBACK && flash_ready && flight_log.initialized)
    {
        // Full erase of a multi-megabyte region at boot can take a long time and
        // trip the task watchdog. Instead, reset the write pointer and erase the
        // first sector now; remaining sectors will be erased on-demand as logging
        // reaches them.
        ESP_LOGW(TAG, "Resetting flight log before logging (FLASH_RESET_BEFORE_READBACK=1)");

        flight_log.write_offset = 0;

        esp_err_t erase_err = flash_memory_erase_4k(flight_log.cfg.base_address);
        if (erase_err != ESP_OK)
            ESP_LOGW(TAG, "Flight log first-sector erase failed: %s", esp_err_to_name(erase_err));
        else
            ESP_LOGI(TAG, "Flight log reset (starting from record 0)");
    }

    mpu6050_init_sensor(&mpu1);
    mpu6050_init_sensor(&mpu2);
    bmp280_init_sensor(&bmp1);
    bmp280_init_sensor(&bmp2);

    // In normal logging mode, print a CSV header once so a serial capture can be
    // saved directly as a CSV file.
    {
        char header[384];
        if (flight_log_format_csv_header(header, sizeof(header)) == ESP_OK)
            log_line_serial_only(header);
    }

    while (1)
    {
        neo6m_loop();
        mpu6050_sample_t m1;
        mpu6050_sample_t m2;
        float b1_alt = NAN;
        float b2_alt = NAN;

        mpu6050_read_sensor(&mpu1, &m1);
        mpu6050_read_sensor(&mpu2, &m2);
        bmp280_read_sensor(&bmp1, &b1_alt);
        bmp280_read_sensor(&bmp2, &b2_alt);

        neo6m_fix_t gps_fix;
        neo6m_get_fix(&gps_fix);

        float gps_lat = NAN;
        float gps_lon = NAN;
        float gps_alt = NAN;
        float gps_sat = NAN;

        if (gps_fix.location_valid &&
            gps_fix.location_age_ms < gps_cfg.max_fix_age_ms)
        {
            gps_lat = (float)gps_fix.lat_deg;
            gps_lon = (float)gps_fix.lon_deg;
        }

        if (gps_fix.altitude_valid)
            gps_alt = gps_fix.altitude_m;

        if (gps_fix.satellites_valid)
            gps_sat = (float)gps_fix.satellites;

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

        .gps_lat_deg = gps_lat,
        .gps_lon_deg = gps_lon,
        .gps_altitude_m = gps_alt,
        .gps_satellites = gps_sat,
    };

        // Always emit the timestamped line to serial (and LoRa if ready).
        {
            char line[384];
            if (flight_log_format_sample_csv(&s, line, sizeof(line)) == ESP_OK)
                write_line_serial_and_lora(NULL, line);
        }

        // And always attempt to save the timestamped sample to flash when available.
        if (flash_ready && flight_log.initialized)
        {
            esp_err_t append_err = flight_log_append(&flight_log, &s);
            if (append_err != ESP_OK)
                ESP_LOGW(TAG, "Flight log append failed: %s", esp_err_to_name(append_err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}