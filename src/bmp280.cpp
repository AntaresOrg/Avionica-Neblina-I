#include "bmp280.h"

#include <math.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_NUM I2C_NUM_0
#define BMP280_ADDR_1 0x76
#define BMP280_ADDR_2 0x77
#define BMP280_CHIP_ID 0x58
#define BME280_CHIP_ID 0x60
#define BMP280_SEA_LEVEL_PA 101325.0f
#define BMP280_BASELINE_STABILIZATION_SAMPLES 10

static const char *TAG = "BMP280";

typedef struct {
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} bmp280_calib_data;

typedef struct {
    uint8_t addr;
    const char *name;
    bmp280_calib_data calib;
    int32_t t_fine;
    uint16_t baseline_samples;
    float baseline_accumulator_m;
    bool baseline_set;
    float baseline_altitude_m;
    bool initialized;
} bmp280_sensor_t;

static bmp280_sensor_t bmp1 = {
    BMP280_ADDR_1,
    "BMP1",
    {},
    0,
    0,
    0.0f,
    false,
    0.0f,
    false,
};
static bmp280_sensor_t bmp2 = {
    BMP280_ADDR_2,
    "BMP2",
    {},
    0,
    0,
    0.0f,
    false,
    0.0f,
    false,
};

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

static float bmp280_pressure_to_altitude(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

static void bmp280_read_calibration(bmp280_sensor_t *sensor)
{
    uint8_t data[24];
    if (i2c_read(sensor->addr, 0x88, data, 24) != ESP_OK)
        return;

    sensor->calib.dig_T1 = (uint16_t)((data[1] << 8) | data[0]);
    sensor->calib.dig_T2 = (int16_t)((data[3] << 8) | data[2]);
    sensor->calib.dig_T3 = (int16_t)((data[5] << 8) | data[4]);

    sensor->calib.dig_P1 = (uint16_t)((data[7] << 8) | data[6]);
    sensor->calib.dig_P2 = (int16_t)((data[9] << 8) | data[8]);
    sensor->calib.dig_P3 = (int16_t)((data[11] << 8) | data[10]);
    sensor->calib.dig_P4 = (int16_t)((data[13] << 8) | data[12]);
    sensor->calib.dig_P5 = (int16_t)((data[15] << 8) | data[14]);
    sensor->calib.dig_P6 = (int16_t)((data[17] << 8) | data[16]);
    sensor->calib.dig_P7 = (int16_t)((data[19] << 8) | data[18]);
    sensor->calib.dig_P8 = (int16_t)((data[21] << 8) | data[20]);
    sensor->calib.dig_P9 = (int16_t)((data[23] << 8) | data[22]);
}

static bool bmp280_check_chip_id(const bmp280_sensor_t *sensor)
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

static void bmp280_init_sensor(bmp280_sensor_t *sensor)
{
    if (!bmp280_check_chip_id(sensor))
    {
        sensor->initialized = false;
        return;
    }

    bmp280_read_calibration(sensor);
    (void)i2c_write(sensor->addr, 0xF4, 0x27);

    sensor->baseline_samples = 0;
    sensor->baseline_accumulator_m = 0.0f;
    sensor->baseline_set = false;
    sensor->baseline_altitude_m = 0.0f;
    sensor->t_fine = 0;
    sensor->initialized = true;
}

static bool bmp280_read_sensor(bmp280_sensor_t *sensor, float *out_relative_altitude_m)
{
    if (out_relative_altitude_m)
        *out_relative_altitude_m = NAN;

    if (!sensor->initialized)
        return false;

    uint8_t d[6];
    if (i2c_read(sensor->addr, 0xF7, d, 6) != ESP_OK)
    {
        ESP_LOGW(TAG, "%s read failed", sensor->name);
        return false;
    }

    int32_t adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);

    int32_t var1 = ((((adc_t >> 3) - ((int32_t)sensor->calib.dig_T1 << 1))) * ((int32_t)sensor->calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_t >> 4) - ((int32_t)sensor->calib.dig_T1)) *
                     ((adc_t >> 4) - ((int32_t)sensor->calib.dig_T1))) >> 12) *
                     ((int32_t)sensor->calib.dig_T3)) >> 14;

    sensor->t_fine = var1 + var2;

    int64_t var1_p = ((int64_t)sensor->t_fine) - 128000;
    int64_t var2_p = var1_p * var1_p * (int64_t)sensor->calib.dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)sensor->calib.dig_P5) << 17);
    var2_p = var2_p + (((int64_t)sensor->calib.dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)sensor->calib.dig_P3) >> 8) +
             ((var1_p * (int64_t)sensor->calib.dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)sensor->calib.dig_P1) >> 33;

    float pressure_pa = 0.0f;
    if (var1_p != 0)
    {
        int64_t p = 1048576 - adc_p;
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)sensor->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = (((int64_t)sensor->calib.dig_P8) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + (((int64_t)sensor->calib.dig_P7) << 4);
        pressure_pa = p / 256.0f;
    }

    if (pressure_pa <= 0.0f)
    {
        ESP_LOGW(TAG, "%s invalid pressure for altitude calculation", sensor->name);
        return false;
    }

    float altitude_m = bmp280_pressure_to_altitude(pressure_pa, BMP280_SEA_LEVEL_PA);

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
        ESP_LOGI(TAG, "%s baseline locked at %.2f m", sensor->name, sensor->baseline_altitude_m);
    }

    float relative_altitude_m = altitude_m - sensor->baseline_altitude_m;
    ESP_LOGI(TAG, "%s Relative Altitude: %.2f m", sensor->name, relative_altitude_m);

    if (out_relative_altitude_m)
        *out_relative_altitude_m = relative_altitude_m;

    return true;
}

void bmp280_init_all(void)
{
    bmp280_init_sensor(&bmp1);
    bmp280_init_sensor(&bmp2);
}

void bmp280_read_all(float *out_bmp1_relative_altitude_m, float *out_bmp2_relative_altitude_m)
{
    bmp280_read_sensor(&bmp1, out_bmp1_relative_altitude_m);
    bmp280_read_sensor(&bmp2, out_bmp2_relative_altitude_m);
}