#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

#define BMP280_ADDR_1 0x76
#define BMP280_ADDR_2 0x77
#define BMP280_CHIP_ID 0x58
#define BMP280_SEA_LEVEL_PA 101325.0f
#define MPU6050_ADDR_1 0x68
#define MPU6050_ADDR_2 0x69
#define MPU6050_WHO_AM_I_REG 0x75

static const char *TAG = "SENSORS";

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
void mpu6050_read_sensor(const mpu6050_sensor_t *sensor)
{
    if (!sensor->initialized)
    {
        return;
    }

    uint8_t d[14];
    if (i2c_read(sensor->addr, 0x3B, d, 14) != ESP_OK)
    {
        ESP_LOGW(TAG, "%s read failed", sensor->name);
        return;
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

    ESP_LOGI(TAG,
             "%s A[g] X=%.2f Y=%.2f Z=%.2f | G[dps] X=%.2f Y=%.2f Z=%.2f",
             sensor->name,
             ax_g,
             ay_g,
             az_g,
             gx_dps,
             gy_dps,
             gz_dps);
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
    bool baseline_set;             /**< True when first altitude baseline was captured. */
    float baseline_altitude_m;     /**< Initial altitude used for relative altitude output. */
    bool initialized;              /**< True when sensor is detected and configured. */
} bmp280_sensor_t;

bmp280_sensor_t bmp1 = {BMP280_ADDR_1, "BMP1", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, false, 0.0f, false};
bmp280_sensor_t bmp2 = {BMP280_ADDR_2, "BMP2", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, 0, false, 0.0f, false};

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
 * @brief Validate BMP280 identity using chip ID register.
 *
 * @param sensor Pointer to target BMP280 descriptor.
 * @return true if chip ID matches BMP280.
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

    if (chip_id != BMP280_CHIP_ID)
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
void bmp280_read_sensor(bmp280_sensor_t *sensor)
{
    if (!sensor->initialized)
    {
        return;
    }

    uint8_t d[6];
    if (i2c_read(sensor->addr, 0xF7, d, 6) != ESP_OK)
    {
        ESP_LOGW(TAG, "%s read failed", sensor->name);
        return;
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
        return;
    }

    float altitude_m = bmp280_pressure_to_altitude(P, BMP280_SEA_LEVEL_PA);

    if (!sensor->baseline_set)
    {
        sensor->baseline_altitude_m = altitude_m;
        sensor->baseline_set = true;
    }

    float relative_altitude_m = altitude_m - sensor->baseline_altitude_m;
    ESP_LOGI(TAG, "%s Relative Altitude: %.2f m", sensor->name, relative_altitude_m);
}

// ---------- MAIN ----------
/**
 * @brief ESP-IDF application entry point.
 *
 * Initializes I2C and all sensors, then continuously reads and logs data.
 */
extern "C" void app_main(void)
{
    i2c_master_init();

    mpu6050_init_sensor(&mpu1);
    mpu6050_init_sensor(&mpu2);
    bmp280_init_sensor(&bmp1);
    bmp280_init_sensor(&bmp2);

    while (1)
    {
        mpu6050_read_sensor(&mpu1);
        mpu6050_read_sensor(&mpu2);
        bmp280_read_sensor(&bmp1);
        bmp280_read_sensor(&bmp2);

        ESP_LOGI(TAG, "------------------------");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}