#include "flight_log.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flash_memory.h"

#define FLIGHT_LOG_MAGIC 0x474F4C46u /* 'FLOG' */
#define FLIGHT_LOG_VERSION 5u

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint16_t version;
    uint16_t length;

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

    float gps_lat_deg;
    float gps_lon_deg;
    float gps_altitude_m;
    float gps_satellites;

    uint32_t crc32;
} flight_log_record_t;

_Static_assert(
    sizeof(flight_log_record_t) ==
    offsetof(flight_log_record_t, crc32) + sizeof(uint32_t),
    "Unexpected flight_log_record_t size");

static uint32_t crc32_ieee(const void *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    const uint8_t *p = (const uint8_t *)data;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= p[i];
        for (int b = 0; b < 8; b++)
        {
            uint32_t mask = (uint32_t)-(int)(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }

    return ~crc;
}

static uint32_t record_address(const flight_log_t *log, uint32_t record_index)
{
    return log->cfg.base_address + record_index * (uint32_t)sizeof(flight_log_record_t);
}

static bool record_is_erased(const flight_log_record_t *rec)
{
    return rec->magic == 0xFFFFFFFFu;
}

static bool record_is_valid(const flight_log_record_t *rec)
{
    if (rec->magic != FLIGHT_LOG_MAGIC)
        return false;
    if (rec->version != FLIGHT_LOG_VERSION)
        return false;
    if (rec->length != (uint16_t)sizeof(flight_sample_t))
        return false;

    uint32_t computed = crc32_ieee(rec, offsetof(flight_log_record_t, crc32));
    return computed == rec->crc32;
}

esp_err_t flight_log_init(flight_log_t *log, const flight_log_config_t *cfg)
{
    if (!log || !cfg)
        return ESP_ERR_INVALID_ARG;
    if (cfg->size_bytes < sizeof(flight_log_record_t))
        return ESP_ERR_INVALID_ARG;
    if ((cfg->size_bytes % 4096) != 0)
        return ESP_ERR_INVALID_ARG;

    memset(log, 0, sizeof(*log));
    log->cfg = *cfg;

    uint32_t max_records = cfg->size_bytes / (uint32_t)sizeof(flight_log_record_t);

    flight_log_record_t rec;
    for (uint32_t i = 0; i < max_records; i++)
    {
        esp_err_t err = flash_memory_read(record_address(log, i), &rec, sizeof(rec));
        if (err != ESP_OK)
            return err;

        if (record_is_erased(&rec))
        {
            log->write_offset = i * (uint32_t)sizeof(flight_log_record_t);
            log->initialized = true;
            return ESP_OK;
        }

        if (!record_is_valid(&rec))
        {
            log->write_offset = i * (uint32_t)sizeof(flight_log_record_t);
            log->initialized = true;
            return ESP_OK;
        }
    }

    log->write_offset = cfg->size_bytes;
    log->initialized = true;
    return ESP_OK;
}

esp_err_t flight_log_erase_all(flight_log_t *log)
{
    if (!log || !log->initialized)
        return ESP_ERR_INVALID_STATE;

    // Erase by 4K sectors.
    for (uint32_t off = 0; off < log->cfg.size_bytes; off += 4096)
    {
        esp_err_t err = flash_memory_erase_4k(log->cfg.base_address + off);
        if (err != ESP_OK)
            return err;
    }

    log->write_offset = 0;
    return ESP_OK;
}

uint32_t flight_log_count(const flight_log_t *log)
{
    if (!log || !log->initialized)
        return 0;
    return log->write_offset / (uint32_t)sizeof(flight_log_record_t);
}

esp_err_t flight_log_append(flight_log_t *log, const flight_sample_t *sample)
{
    if (!log || !log->initialized)
        return ESP_ERR_INVALID_STATE;
    if (!sample)
        return ESP_ERR_INVALID_ARG;

    if (log->write_offset + sizeof(flight_log_record_t) > log->cfg.size_bytes)
        return ESP_ERR_NO_MEM;

    // Ensure the target 4KB sector is erased before writing into it.
    // This makes it safe to "reset" the log by setting write_offset=0 without
    // erasing the whole region up front.
    if ((log->write_offset % 4096u) == 0)
    {
        uint32_t sector_addr = log->cfg.base_address + log->write_offset;
        bool has_data = false;
        esp_err_t err = flash_memory_sector_has_data_4k(sector_addr, &has_data);
        if (err != ESP_OK)
            return err;

        if (has_data)
        {
            err = flash_memory_erase_4k(sector_addr);
            if (err != ESP_OK)
                return err;
        }
    }

    flight_log_record_t rec;
    memset(&rec, 0, sizeof(rec));

    rec.magic = FLIGHT_LOG_MAGIC;
    rec.version = FLIGHT_LOG_VERSION;
    rec.length = (uint16_t)sizeof(flight_sample_t);

    rec.time_ms = sample->time_ms;
    rec.bmp1_relative_altitude_m = sample->bmp1_relative_altitude_m;
    rec.bmp2_relative_altitude_m = sample->bmp2_relative_altitude_m;

    rec.mpu1_ax_g = sample->mpu1_ax_g;
    rec.mpu1_ay_g = sample->mpu1_ay_g;
    rec.mpu1_az_g = sample->mpu1_az_g;
    rec.mpu1_gx_dps = sample->mpu1_gx_dps;
    rec.mpu1_gy_dps = sample->mpu1_gy_dps;
    rec.mpu1_gz_dps = sample->mpu1_gz_dps;

    rec.mpu2_ax_g = sample->mpu2_ax_g;
    rec.mpu2_ay_g = sample->mpu2_ay_g;
    rec.mpu2_az_g = sample->mpu2_az_g;
    rec.mpu2_gx_dps = sample->mpu2_gx_dps;
    rec.mpu2_gy_dps = sample->mpu2_gy_dps;
    rec.mpu2_gz_dps = sample->mpu2_gz_dps;

    rec.gps_lat_deg = sample->gps_lat_deg;
    rec.gps_lon_deg = sample->gps_lon_deg;
    rec.gps_altitude_m = sample->gps_altitude_m;
    rec.gps_satellites = sample->gps_satellites;

    rec.crc32 = crc32_ieee(&rec, offsetof(flight_log_record_t, crc32));

    esp_err_t err = flash_memory_write(log->cfg.base_address + log->write_offset, &rec, sizeof(rec));
    if (err != ESP_OK)
        return err;

    log->write_offset += (uint32_t)sizeof(flight_log_record_t);
    return ESP_OK;
}

esp_err_t flight_log_read(const flight_log_t *log, uint32_t index, flight_sample_t *out)
{
    if (!log || !log->initialized)
        return ESP_ERR_INVALID_STATE;
    if (!out)
        return ESP_ERR_INVALID_ARG;

    uint32_t count = flight_log_count(log);
    if (index >= count)
        return ESP_ERR_NOT_FOUND;

    flight_log_record_t rec;
    esp_err_t err = flash_memory_read(record_address(log, index), &rec, sizeof(rec));
    if (err != ESP_OK)
        return err;

    if (!record_is_valid(&rec))
        return ESP_ERR_INVALID_CRC;

    out->time_ms = rec.time_ms;
    out->bmp1_relative_altitude_m = rec.bmp1_relative_altitude_m;
    out->bmp2_relative_altitude_m = rec.bmp2_relative_altitude_m;

    out->mpu1_ax_g = rec.mpu1_ax_g;
    out->mpu1_ay_g = rec.mpu1_ay_g;
    out->mpu1_az_g = rec.mpu1_az_g;
    out->mpu1_gx_dps = rec.mpu1_gx_dps;
    out->mpu1_gy_dps = rec.mpu1_gy_dps;
    out->mpu1_gz_dps = rec.mpu1_gz_dps;

    out->mpu2_ax_g = rec.mpu2_ax_g;
    out->mpu2_ay_g = rec.mpu2_ay_g;
    out->mpu2_az_g = rec.mpu2_az_g;
    out->mpu2_gx_dps = rec.mpu2_gx_dps;
    out->mpu2_gy_dps = rec.mpu2_gy_dps;
    out->mpu2_gz_dps = rec.mpu2_gz_dps;

    out->gps_lat_deg = rec.gps_lat_deg;
    out->gps_lon_deg = rec.gps_lon_deg;
    out->gps_altitude_m = rec.gps_altitude_m;
    out->gps_satellites = rec.gps_satellites;

    return ESP_OK;
}

static const char *flight_log_csv_header_line(void)
{
    return "time_ms,bmp1_rel_alt_m,bmp2_rel_alt_m,"
           "mpu1_ax_g,mpu1_ay_g,mpu1_az_g,mpu1_gx_dps,mpu1_gy_dps,mpu1_gz_dps,"
           "mpu2_ax_g,mpu2_ay_g,mpu2_az_g,mpu2_gx_dps,mpu2_gy_dps,mpu2_gz_dps,"
           "gps_lat_deg,gps_lon_deg,gps_alt_m,gps_sat";
}

esp_err_t flight_log_format_csv_header(char *out, size_t out_len)
{
    if (!out || out_len == 0)
        return ESP_ERR_INVALID_ARG;

    int n = snprintf(out, out_len, "%s", flight_log_csv_header_line());
    if (n < 0 || (size_t)n >= out_len)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t flight_log_format_sample_csv(const flight_sample_t *sample, char *out, size_t out_len)
{
    if (!sample || !out || out_len == 0)
        return ESP_ERR_INVALID_ARG;

    int n = snprintf(
        out,
        out_len,
        "%u,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.6f,%.1f,%.0f",
        (unsigned int)sample->time_ms,
        sample->bmp1_relative_altitude_m,
        sample->bmp2_relative_altitude_m,
        sample->mpu1_ax_g,
        sample->mpu1_ay_g,
        sample->mpu1_az_g,
        sample->mpu1_gx_dps,
        sample->mpu1_gy_dps,
        sample->mpu1_gz_dps,
        sample->mpu2_ax_g,
        sample->mpu2_ay_g,
        sample->mpu2_az_g,
        sample->mpu2_gx_dps,
        sample->mpu2_gy_dps,
        sample->mpu2_gz_dps,
        sample->gps_lat_deg,
        sample->gps_lon_deg,
        sample->gps_altitude_m,
        sample->gps_satellites);

    if (n < 0 || (size_t)n >= out_len)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}

esp_err_t flight_log_dump_csv(const flight_log_t *log,
                              flight_log_write_line_fn write_line,
                              void *ctx,
                              bool include_header)
{
    if (!log || !log->initialized)
        return ESP_ERR_INVALID_STATE;
    if (!write_line)
        return ESP_ERR_INVALID_ARG;

    char line[384];

    if (include_header)
    {
        esp_err_t err = flight_log_format_csv_header(line, sizeof(line));
        if (err != ESP_OK)
            return err;
        write_line(ctx, line);
    }

    uint32_t count = flight_log_count(log);
    for (uint32_t i = 0; i < count; i++)
    {
        flight_sample_t sample;
        esp_err_t err = flight_log_read(log, i, &sample);
        if (err != ESP_OK)
            return err;

        err = flight_log_format_sample_csv(&sample, line, sizeof(line));
        if (err != ESP_OK)
            return err;

        write_line(ctx, line);
    }

    return ESP_OK;
}
