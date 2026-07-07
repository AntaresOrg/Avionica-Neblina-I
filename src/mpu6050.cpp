#include "mpu6050.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define MPU6050_ADDR_1 0x68
#define MPU6050_ADDR_2 0x69
#define MPU6050_WHO_AM_I_REG 0x75

static const char *TAG = "MPU6050";

typedef struct {
    uint8_t addr;
    const char *name;
    bool initialized;
} mpu6050_sensor_t;

static mpu6050_sensor_t mpu1 = {
    .addr = MPU6050_ADDR_1,
    .name = "MPU1",
    .initialized = false,
};

static mpu6050_sensor_t mpu2 = {
    .addr = MPU6050_ADDR_2,
    .name = "MPU2",
    .initialized = false,
};

static void publish_line(const char *fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);

    ESP_LOGI(TAG, "%s", line);
}

static esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

static esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, addr, buf, 2, pdMS_TO_TICKS(100));
}

static void mpu6050_init_sensor(mpu6050_sensor_t *sensor)
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

static bool mpu6050_read_sensor(const mpu6050_sensor_t *sensor, mpu6050_sample_t *out)
{
    if (out)
        memset(out, 0, sizeof(*out));

    if (!sensor->initialized)
        return false;

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

    float ax_g = ax / 16384.0f;
    float ay_g = ay / 16384.0f;
    float az_g = az / 16384.0f;

    float gx_dps = gx / 131.0f;
    float gy_dps = gy / 131.0f;
    float gz_dps = gz / 131.0f;

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

void mpu6050_init_all(void)
{
    mpu6050_init_sensor(&mpu1);
    mpu6050_init_sensor(&mpu2);
}

void mpu6050_read_all(mpu6050_sample_t *out_mpu1, mpu6050_sample_t *out_mpu2)
{
    mpu6050_read_sensor(&mpu1, out_mpu1);
    mpu6050_read_sensor(&mpu2, out_mpu2);
}

bool mpu6050_is_initialized(size_t sensor_index)
{
    if (sensor_index == 0)
        return mpu1.initialized;
    if (sensor_index == 1)
        return mpu2.initialized;

    return false;
}