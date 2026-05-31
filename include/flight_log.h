#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t time_ms;

    float bmp1_relative_altitude_m;
    float bmp2_relative_altitude_m;

    float mpu1_ax_g;
    float mpu1_ay_g;
    float mpu1_az_g;
    float mpu1_gx_dps;
    float mpu1_gy_dps;
    float mpu1_gz_dps;

    float mpu2_ax_g;
    float mpu2_ay_g;
    float mpu2_az_g;
    float mpu2_gx_dps;
    float mpu2_gy_dps;
    float mpu2_gz_dps;

    // GPS (Neo-6M) fields. Use NAN when invalid/unavailable.
    float gps_lat_deg;
    float gps_lon_deg;
    float gps_altitude_m;
    float gps_satellites;

} flight_sample_t;

typedef struct {
    uint32_t base_address;
    uint32_t size_bytes;
} flight_log_config_t;

typedef struct {
    flight_log_config_t cfg;
    uint32_t write_offset;
    bool initialized;
} flight_log_t;

typedef void (*flight_log_write_line_fn)(void *ctx, const char *line);

esp_err_t flight_log_init(flight_log_t *log, const flight_log_config_t *cfg);

// Erases the whole configured log area (slow, but simplest for "start new flight").
esp_err_t flight_log_erase_all(flight_log_t *log);

// Appends one sample. Requires enough remaining space.
esp_err_t flight_log_append(flight_log_t *log, const flight_sample_t *sample);

// Number of valid records currently present.
uint32_t flight_log_count(const flight_log_t *log);

// Reads a sample by index [0..count-1].
esp_err_t flight_log_read(const flight_log_t *log, uint32_t index, flight_sample_t *out);

// ---- CSV helpers (formatting + dump) ----
// Fills `out` with the CSV header line.
esp_err_t flight_log_format_csv_header(char *out, size_t out_len);

// Fills `out` with one CSV line for a sample.
esp_err_t flight_log_format_sample_csv(const flight_sample_t *sample, char *out, size_t out_len);

// Dumps [0..count-1] records to the provided line writer in CSV format.
// If include_header is true, writes the header first.
esp_err_t flight_log_dump_csv(const flight_log_t *log,
                              flight_log_write_line_fn write_line,
                              void *ctx,
                              bool include_header);

#ifdef __cplusplus
}
#endif
