#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <limits.h>
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

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define BMP280_ADDR_1 0x76
#define BMP280_ADDR_2 0x77
#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60
#define BMP280_SEA_LEVEL_PA 101325.0f
#define BMP280_BASELINE_STABILIZATION_SAMPLES 10
#define MPU6050_ADDR_1 0x68
#define MPU6050_ADDR_2 0x69
#define MPU6050_WHO_AM_I_REG 0x75

static const char *TAG = "SENSORS";

// Set to 1 to boot into flash readback (dump CSV), 0 for normal logging.
#define FLASH_READBACK_MODE 0
// Set to 1 to erase the flight log region on boot BEFORE logging starts.
// This runs only in normal logging mode (FLASH_READBACK_MODE=0).
#define FLASH_RESET_BEFORE_READBACK 0

// Minimum interval between LoRa transmissions of flight-log packets.
#define LORA_TX_INTERVAL_MS 1000u

typedef enum {
    TX_MODE_CSV = 0,
    TX_MODE_STRUCT = 1,
} telemetry_tx_mode_t;

// Set transmission mode for LoRa packets.
#define TELEMETRY_TX_MODE TX_MODE_CSV

typedef struct __attribute__((packed)) {
    int16_t tempo_s_x1;
    int16_t bmp1_alt_rel_m_x10;
    int16_t bmp2_alt_rel_m_x10;

    int16_t mpu1_lin_x_x100;
    int16_t mpu1_lin_y_x100;
    int16_t mpu1_lin_z_x100;
    int16_t mpu1_rad_x_x100;
    int16_t mpu1_rad_y_x100;
    int16_t mpu1_rad_z_x100;

    int16_t mpu2_lin_x_x100;
    int16_t mpu2_lin_y_x100;
    int16_t mpu2_lin_z_x100;
    int16_t mpu2_rad_x_x100;
    int16_t mpu2_rad_y_x100;
    int16_t mpu2_rad_z_x100;

    char gps_lat[6];
    char gps_lon[6];
    int16_t gps_alt_m_x10;
    uint8_t gps_sat_count_x1;

    // Reserved/padding to keep LoRa binary payload fixed at 64 bytes.
    uint8_t reserved[19];
} telemetry_packet_t;

static_assert(sizeof(telemetry_packet_t) == 64, "Unexpected telemetry_packet_t size");

static bool lora_ready = false;
static bool flash_ready = false;
static flight_log_t flight_log;
static uint32_t last_lora_tx_ms = 0;

static void log_line_serial_only(const char *line);

static int16_t scale_to_i16(float value, float factor)
{
    if (isnan(value))
        return 0;

    float scaled = value * factor;

    if (scaled > (float)INT16_MAX)
        return INT16_MAX;
    if (scaled < (float)INT16_MIN)
        return INT16_MIN;

    return (int16_t)scaled;
}

static uint8_t scale_to_u8(float value)
{
    if (isnan(value) || value < 0.0f)
        return 0;
    if (value > 255.0f)
        return 255;

    return (uint8_t)value;
}

static void encode_deg_x100_to_6chars(float deg, char out[6])
{
    if (isnan(deg))
    {
        memcpy(out, "------", 6);
        return;
    }

    int scaled = (int)(deg * 100.0f);
    if (scaled > 99999)
        scaled = 99999;
    if (scaled < -99999)
        scaled = -99999;

    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%+06d", scaled);
    memcpy(out, tmp, 6);
}

static telemetry_packet_t build_telemetry_packet(const flight_sample_t *s)
{
    telemetry_packet_t p = {};

    if (!s)
        return p;

    p.tempo_s_x1 = scale_to_i16((float)s->time_ms / 1000.0f, 1.0f);
    p.bmp1_alt_rel_m_x10 = scale_to_i16(s->bmp1_relative_altitude_m, 10.0f);
    p.bmp2_alt_rel_m_x10 = scale_to_i16(s->bmp2_relative_altitude_m, 10.0f);

    p.mpu1_lin_x_x100 = scale_to_i16(s->mpu1_ax_g, 100.0f);
    p.mpu1_lin_y_x100 = scale_to_i16(s->mpu1_ay_g, 100.0f);
    p.mpu1_lin_z_x100 = scale_to_i16(s->mpu1_az_g, 100.0f);
    p.mpu1_rad_x_x100 = scale_to_i16(s->mpu1_gx_dps, 100.0f);
    p.mpu1_rad_y_x100 = scale_to_i16(s->mpu1_gy_dps, 100.0f);
    p.mpu1_rad_z_x100 = scale_to_i16(s->mpu1_gz_dps, 100.0f);

    p.mpu2_lin_x_x100 = scale_to_i16(s->mpu2_ax_g, 100.0f);
    p.mpu2_lin_y_x100 = scale_to_i16(s->mpu2_ay_g, 100.0f);
    p.mpu2_lin_z_x100 = scale_to_i16(s->mpu2_az_g, 100.0f);
    p.mpu2_rad_x_x100 = scale_to_i16(s->mpu2_gx_dps, 100.0f);
    p.mpu2_rad_y_x100 = scale_to_i16(s->mpu2_gy_dps, 100.0f);
    p.mpu2_rad_z_x100 = scale_to_i16(s->mpu2_gz_dps, 100.0f);

    encode_deg_x100_to_6chars(s->gps_lat_deg, p.gps_lat);
    encode_deg_x100_to_6chars(s->gps_lon_deg, p.gps_lon);
    p.gps_alt_m_x10 = scale_to_i16(s->gps_altitude_m, 10.0f);
    p.gps_sat_count_x1 = scale_to_u8(s->gps_satellites);

    return p;
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

typedef struct {
    bool valid;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} mpu6050_sample_t;

static void publish_line(const char *fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    ESP_LOGI(TAG, "%s", line);
    // LoRa transmission is reserved for flash-backed flight-log packets.
}

static void log_line_serial_only(const char *line)
{
    if (!line)
        return;

    // Plain UART0 output (serial monitor friendly; no ESP_LOG prefixes).
    printf("%s\n", line);
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
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ---------- I2C ----------
/**
 * @brief Read bytes from a register in an I2C device.
 *
 * @param addr 7-bit I2C address of the target device.
 * @param reg Register address to start reading from.
 * @param data Output buffer where read bytes are stored.
 * @param len Number of bytes to read.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/**
 * @brief Write one byte to an I2C device register.
 *
 * @param addr 7-bit I2C address of the target device.
 * @param reg Register address to write.
 * @param data Byte value to write.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, addr, buf, 2, pdMS_TO_TICKS(100));
}

// ---------- MPU6050 ----------
/**
 * @brief Runtime state for one MPU6050 sensor.
 */
typedef struct {
    uint8_t addr;          /**< I2C address (0x68 or 0x69). */
    const char *name;      /**< Label used in logs. */
    bool initialized;      /**< True when sensor is detected and configured. */
} mpu6050_sensor_t;

mpu6050_sensor_t mpu1 = {
    .addr = MPU6050_ADDR_1,
    .name = "MPU1",
    .initialized = false,
};

mpu6050_sensor_t mpu2 = {
    .addr = MPU6050_ADDR_2,
    .name = "MPU2",
    .initialized = false,
};

/**
 * @brief Initialize one MPU6050 sensor.
 *
 * Reads WHO_AM_I for basic communication validation and then clears sleep mode
 * by writing 0x00 to register 0x6B.
 *
 * @param sensor Pointer to the MPU6050 descriptor to initialize.
 */
void mpu6050_init_sensor(mpu6050_sensor_t *sensor)
{
    uint8_t who_am_i = 0;

    if (i2c_read(sensor->addr, MPU6050_WHO_AM_I_REG, &who_am_i, 1) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s WHO_AM_I read failed (addr 0x%02X)", sensor->name, sensor->addr);
        sensor->initialized = false;
        return;
    }

    if (who_am_i != MPU6050_ADDR_1 && who_am_i != MPU6050_ADDR_2)
    {
        ESP_LOGE(TAG, "%s unexpected WHO_AM_I: 0x%02X", sensor->name, who_am_i);
        sensor->initialized = false;
        return;
    }

    if (i2c_write(sensor->addr, 0x6B, 0x00) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s wake-up write failed", sensor->name);
        sensor->initialized = false;
        return;
    }

    sensor->initialized = true;
}

/**
 * @brief Convert pressure in Pascal to altitude in meters.
 *
 * @param pressure_pa Measured pressure in Pascal.
 * @param sea_level_pa Reference sea-level pressure in Pascal.
 * @return Altitude in meters.
 */
float bmp280_pressure_to_altitude(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

/**
 * @brief Read one MPU6050 sample and print one compact log line.
 *
 * Reads 14 bytes from ACCEL_XOUT_H (0x3B), converts accelerometer to g and gyroscope
 * to deg/s, then logs all six values in one line for this sensor.
 *
 * @param sensor Pointer to the initialized sensor descriptor.
 */
bool mpu6050_read_sensor(const mpu6050_sensor_t *sensor, mpu6050_sample_t *out)
{
    if (out)
        memset(out, 0, sizeof(*out));

    if (!sensor->initialized)
    {
        return false;
    }

    uint8_t d[14];
    if (i2c_read(sensor->addr, 0x3B, d, 14) != ESP_OK)
    {
        ESP_LOGW(TAG, "%s read failed", sensor->name);
        return false;
    }

    int16_t ax = (d[0] << 8) | d[1];
    int16_t ay = (d[2] << 8) | d[3];
    int16_t az = (d[4] << 8) | d[5];
    int16_t gx = (d[8] << 8) | d[9];
    int16_t gy = (d[10] << 8) | d[11];
    int16_t gz = (d[12] << 8) | d[13];

    // Convert to real units
    float ax_g = ax / 16384.0;
    float ay_g = ay / 16384.0;
    float az_g = az / 16384.0;

    float gx_dps = gx / 131.0;
    float gy_dps = gy / 131.0;
    float gz_dps = gz / 131.0;

    if (out)
    {
        out->valid = true;
        out->ax_g = ax_g;
        out->ay_g = ay_g;
        out->az_g = az_g;
        out->gx_dps = gx_dps;
        out->gy_dps = gy_dps;
        out->gz_dps = gz_dps;
    }

    publish_line(
        "%s A[g] X=%.2f Y=%.2f Z=%.2f | G[dps] X=%.2f Y=%.2f Z=%.2f",
        sensor->name,
        ax_g,
        ay_g,
        az_g,
        gx_dps,
        gy_dps,
        gz_dps);

    return true;
}

// ---------- BMP280 ----------
/**
 * @brief Factory calibration coefficients read from BMP280 NVM.
 */
typedef struct {
    uint16_t dig_T1;       /**< Temperature calibration term T1. */
    int16_t dig_T2;        /**< Temperature calibration term T2. */
    int16_t dig_T3;        /**< Temperature calibration term T3. */
    uint16_t dig_P1;       /**< Pressure calibration term P1. */
    int16_t dig_P2;        /**< Pressure calibration term P2. */
    int16_t dig_P3;        /**< Pressure calibration term P3. */
    int16_t dig_P4;        /**< Pressure calibration term P4. */
    int16_t dig_P5;        /**< Pressure calibration term P5. */
    int16_t dig_P6;        /**< Pressure calibration term P6. */
    int16_t dig_P7;        /**< Pressure calibration term P7. */
    int16_t dig_P8;        /**< Pressure calibration term P8. */
    int16_t dig_P9;        /**< Pressure calibration term P9. */
} bmp280_calib_data;

/**
 * @brief Runtime state for one BMP280 sensor.
 */
typedef struct {
    uint8_t addr;                  /**< I2C address (0x76 or 0x77). */
    const char *name;              /**< Label used in logs. */
    bmp280_calib_data calib;       /**< Cached factory compensation coefficients. */
    int32_t t_fine;                /**< Intermediate variable required by BMP280 formulas. */
    uint16_t baseline_samples;      /**< Number of altitude samples collected for baseline. */
    float baseline_accumulator_m;   /**< Running sum used to average baseline altitude. */
    bool baseline_set;             /**< True when first altitude baseline was captured. */
    float baseline_altitude_m;     /**< Initial altitude used for relative altitude output. */
    bool initialized;              /**< True when sensor is detected and configured. */
} bmp280_sensor_t;

bmp280_sensor_t bmp1 = {BMP280_ADDR_1, "BMP1", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0.0f, false, 0.0f, false};
bmp280_sensor_t bmp2 = {BMP280_ADDR_2, "BMP2", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, 0, 0.0f, false, 0.0f, false};

/**
 * @brief Read BMP280 factory calibration registers into sensor state.
 *
 * @param sensor Pointer to target BMP280 descriptor.
 */
void bmp280_read_calibration(bmp280_sensor_t *sensor)
{
    uint8_t data[24];
    i2c_read(sensor->addr, 0x88, data, 24);

    sensor->calib.dig_T1 = (data[1] << 8) | data[0];
    sensor->calib.dig_T2 = (data[3] << 8) | data[2];
    sensor->calib.dig_T3 = (data[5] << 8) | data[4];

    sensor->calib.dig_P1 = (data[7] << 8) | data[6];
    sensor->calib.dig_P2 = (data[9] << 8) | data[8];
    sensor->calib.dig_P3 = (data[11] << 8) | data[10];
    sensor->calib.dig_P4 = (data[13] << 8) | data[12];
    sensor->calib.dig_P5 = (data[15] << 8) | data[14];
    sensor->calib.dig_P6 = (data[17] << 8) | data[16];
    sensor->calib.dig_P7 = (data[19] << 8) | data[18];
    sensor->calib.dig_P8 = (data[21] << 8) | data[20];
    sensor->calib.dig_P9 = (data[23] << 8) | data[22];
}

/**
 * @brief Validate BMP/BME280 identity using chip ID register.
 *
 * @param sensor Pointer to target BMP280 descriptor.
 * @return true if chip ID matches BMP280 (0x58) or BME280 (0x60).
 * @return false if the read fails or chip ID is unexpected.
 */
bool bmp280_check_chip_id(const bmp280_sensor_t *sensor)
{
    uint8_t chip_id = 0;
    if (i2c_read(sensor->addr, 0xD0, &chip_id, 1) != ESP_OK)
    {
        ESP_LOGE(TAG, "%s chip ID read failed (addr 0x%02X)", sensor->name, sensor->addr);
        return false;
    }

    if (chip_id != BMP280_CHIP_ID && chip_id != BME280_CHIP_ID)
    {
        ESP_LOGE(TAG, "%s unexpected chip ID: 0x%02X", sensor->name, chip_id);
        return false;
    }

    return true;
}

/**
 * @brief Initialize one BMP280 sensor.
 *
 * Verifies chip ID, reads calibration coefficients, and sets control register
 * 0xF4 to 0x27 for normal periodic measurements.
 *
 * @param sensor Pointer to target BMP280 descriptor.
 */
void bmp280_init_sensor(bmp280_sensor_t *sensor)
{
    if (!bmp280_check_chip_id(sensor))
    {
        sensor->initialized = false;
        return;
    }

    bmp280_read_calibration(sensor);
    i2c_write(sensor->addr, 0xF4, 0x27);

    sensor->baseline_samples = 0;
    sensor->baseline_accumulator_m = 0.0f;
    sensor->baseline_set = false;
    sensor->baseline_altitude_m = 0.0f;

    sensor->initialized = true;
}

/**
 * @brief Read one BMP280 sample and print relative altitude.
 *
 * Computes compensated pressure and relative altitude. The first valid altitude
 * sample becomes the baseline (0 m), and subsequent outputs are shown relative
 * to that first sample.
 *
 * @param sensor Pointer to initialized BMP280 descriptor.
 */
bool bmp280_read_sensor(bmp280_sensor_t *sensor, float *out_relative_altitude_m)
{
    if (out_relative_altitude_m)
        *out_relative_altitude_m = NAN;

    if (!sensor->initialized)
    {
        return false;
    }

    uint8_t d[6];
    if (i2c_read(sensor->addr, 0xF7, d, 6) != ESP_OK)
    {
        ESP_LOGW(TAG, "%s read failed", sensor->name);
        return false;
    }

    int32_t adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);

    // Temperature compensation
        int32_t var1 = ((((adc_t >> 3) - ((int32_t)sensor->calib.dig_T1 << 1))) * ((int32_t)sensor->calib.dig_T2)) >> 11;
        int32_t var2 = (((((adc_t >> 4) - ((int32_t)sensor->calib.dig_T1)) *
                                            ((adc_t >> 4) - ((int32_t)sensor->calib.dig_T1))) >> 12) *
                                        ((int32_t)sensor->calib.dig_T3)) >> 14;

        sensor->t_fine = var1 + var2;
        float T = (sensor->t_fine * 5 + 128) >> 8;
    T /= 100.0;

    // Pressure compensation
        int64_t var1_p = ((int64_t)sensor->t_fine) - 128000;
        int64_t var2_p = var1_p * var1_p * (int64_t)sensor->calib.dig_P6;
        var2_p = var2_p + ((var1_p * (int64_t)sensor->calib.dig_P5) << 17);
        var2_p = var2_p + (((int64_t)sensor->calib.dig_P4) << 35);
        var1_p = ((var1_p * var1_p * (int64_t)sensor->calib.dig_P3) >> 8) +
                         ((var1_p * (int64_t)sensor->calib.dig_P2) << 12);
        var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)sensor->calib.dig_P1) >> 33;

    float P = 0;
    if (var1_p != 0)
    {
        int64_t p = 1048576 - adc_p;
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)sensor->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = (((int64_t)sensor->calib.dig_P8) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + (((int64_t)sensor->calib.dig_P7) << 4);
        P = p / 256.0;
    }

    if (P <= 0.0f)
    {
        ESP_LOGW(TAG, "%s invalid pressure for altitude calculation", sensor->name);
        return false;
    }

    float altitude_m = bmp280_pressure_to_altitude(P, BMP280_SEA_LEVEL_PA);

    if (!sensor->baseline_set)
    {
        sensor->baseline_accumulator_m += altitude_m;
        sensor->baseline_samples++;

        if (sensor->baseline_samples < BMP280_BASELINE_STABILIZATION_SAMPLES)
        {
            ESP_LOGI(TAG,
                     "%s stabilizing baseline (%u/%u)",
                     sensor->name,
                     (unsigned int)sensor->baseline_samples,
                     (unsigned int)BMP280_BASELINE_STABILIZATION_SAMPLES);
            return false;
        }

        sensor->baseline_altitude_m = sensor->baseline_accumulator_m / sensor->baseline_samples;
        sensor->baseline_set = true;
        publish_line("%s baseline locked at %.2f m", sensor->name, sensor->baseline_altitude_m);
    }

    float relative_altitude_m = altitude_m - sensor->baseline_altitude_m;
    publish_line("%s Relative Altitude: %.2f m", sensor->name, relative_altitude_m);

    if (out_relative_altitude_m)
        *out_relative_altitude_m = relative_altitude_m;

    return true;
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
        // D1 = MOSI
        .mosi_io_num = GPIO_NUM_23,
        // D0 = MISO
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
        poll_lora_rx();
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

        // Always emit the timestamped line to serial.
        // LoRa should transmit only packets that were successfully saved to flash.
        char line[384];
        bool line_ok = (flight_log_format_sample_csv(&s, line, sizeof(line)) == ESP_OK);
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
            if (last_lora_tx_ms == 0 || (uint32_t)(now_ms - last_lora_tx_ms) >= LORA_TX_INTERVAL_MS)
            {
                if (TELEMETRY_TX_MODE == TX_MODE_STRUCT)
                {
                    telemetry_packet_t packet = build_telemetry_packet(&s);
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