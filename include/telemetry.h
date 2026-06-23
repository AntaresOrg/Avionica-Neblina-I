#pragma once

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "flight_log.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TX_MODE_CSV = 0,
    TX_MODE_STRUCT = 1,
} telemetry_tx_mode_t;

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

    uint8_t reserved[19];
} telemetry_packet_t;

static inline void telemetry_packet_size_check(void)
{
    _Static_assert(sizeof(telemetry_packet_t) == 64, "Unexpected telemetry_packet_t size");
}

telemetry_packet_t telemetry_build_packet(const flight_sample_t *sample);

esp_err_t telemetry_format_sample_csv(const flight_sample_t *sample, char *out, size_t out_len);

#ifdef __cplusplus
}
#endif