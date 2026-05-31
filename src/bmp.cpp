#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdint.h>
#include "i2c.h"
#include "bmp.h"

static const char *TAG = "BMP280";

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


// ---------- BMP280 ----------
/**
 * @brief Factory calibration coefficients read from BMP280 NVM.
 */

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
        sensor->baseline_accumulator_m += altitude_m;
        sensor->baseline_samples++;

        if (sensor->baseline_samples < BMP280_BASELINE_STABILIZATION_SAMPLES)
        {
            ESP_LOGI(TAG,
                     "%s stabilizing baseline (%u/%u)",
                     sensor->name,
                     (unsigned int)sensor->baseline_samples,
                     (unsigned int)BMP280_BASELINE_STABILIZATION_SAMPLES);
            return;
        }

        sensor->baseline_altitude_m = sensor->baseline_accumulator_m / sensor->baseline_samples;
        sensor->baseline_set = true;
        publish_line("%s baseline locked at %.2f m", sensor->name, sensor->baseline_altitude_m);
    }

    float relative_altitude_m = altitude_m - sensor->baseline_altitude_m;
    publish_line("%s Relative Altitude: %.2f m", sensor->name, relative_altitude_m);
}