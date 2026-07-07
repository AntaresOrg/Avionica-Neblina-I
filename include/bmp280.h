#pragma once

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void bmp280_init_all(void);

void bmp280_read_all(float *out_bmp1_relative_altitude_m, float *out_bmp2_relative_altitude_m);

bool bmp280_is_initialized(size_t sensor_index);

#ifdef __cplusplus
}
#endif