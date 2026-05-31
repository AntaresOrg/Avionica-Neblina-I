#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "i2c.h"
#include "mpu6050.h"

static const char *TAG = "MPU6050";

// ---------- MPU6050 ----------
/**
 * @brief Runtime state for one MPU6050 sensor.
 */

/**
 * @brief Initialize one MPU6050 sensor.
 *
 * Reads WHO_AM_I for basic communication validation and then clears sleep mode
 * by writing 0x00 to register 0x6B.
 *
 * @param sensor Pointer to the MPU6050 descriptor to initialize.
 */

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

extern void publish_line(const char *fmt, ...);

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
