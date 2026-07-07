#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} mpu6050_sample_t;

void mpu6050_init_all(void);

void mpu6050_read_all(mpu6050_sample_t *out_mpu1, mpu6050_sample_t *out_mpu2);

bool mpu6050_is_initialized(size_t sensor_index);

#ifdef __cplusplus
}
#endif