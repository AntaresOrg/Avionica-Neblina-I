#include "telemetry.h"

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static int16_t scale_to_i16(float value, float factor)
{
    if (isnan(value))
        return 0;

    float scaled = value * factor;

    if (scaled > (float)INT16_MAX)
        return INT16_MAX;
    if (scaled < (float)INT16_MIN)
        return INT16_MIN;

    return (int16_t)scaled;
}

static uint8_t scale_to_u8(float value)
{
    if (isnan(value) || value < 0.0f)
        return 0;
    if (value > 255.0f)
        return 255;

    return (uint8_t)value;
}

static void encode_deg_x100_to_6chars(float deg, char out[6])
{
    if (isnan(deg))
    {
        memcpy(out, "------", 6);
        return;
    }

    int scaled = (int)(deg * 100.0f);
    if (scaled > 99999)
        scaled = 99999;
    if (scaled < -99999)
        scaled = -99999;

    char tmp[8];
    snprintf(tmp, sizeof(tmp), "%+06d", scaled);
    memcpy(out, tmp, 6);
}

telemetry_packet_t telemetry_build_packet(const flight_sample_t *sample)
{
    telemetry_packet_t packet = {};

    if (!sample)
        return packet;

    packet.tempo_s_x1 = scale_to_i16((float)sample->time_ms / 1000.0f, 1.0f);
    packet.bmp1_alt_rel_m_x10 = scale_to_i16(sample->bmp1_relative_altitude_m, 10.0f);
    packet.bmp2_alt_rel_m_x10 = scale_to_i16(sample->bmp2_relative_altitude_m, 10.0f);

    packet.mpu1_lin_x_x100 = scale_to_i16(sample->mpu1_ax_g, 100.0f);
    packet.mpu1_lin_y_x100 = scale_to_i16(sample->mpu1_ay_g, 100.0f);
    packet.mpu1_lin_z_x100 = scale_to_i16(sample->mpu1_az_g, 100.0f);
    packet.mpu1_rad_x_x100 = scale_to_i16(sample->mpu1_gx_dps, 100.0f);
    packet.mpu1_rad_y_x100 = scale_to_i16(sample->mpu1_gy_dps, 100.0f);
    packet.mpu1_rad_z_x100 = scale_to_i16(sample->mpu1_gz_dps, 100.0f);

    packet.mpu2_lin_x_x100 = scale_to_i16(sample->mpu2_ax_g, 100.0f);
    packet.mpu2_lin_y_x100 = scale_to_i16(sample->mpu2_ay_g, 100.0f);
    packet.mpu2_lin_z_x100 = scale_to_i16(sample->mpu2_az_g, 100.0f);
    packet.mpu2_rad_x_x100 = scale_to_i16(sample->mpu2_gx_dps, 100.0f);
    packet.mpu2_rad_y_x100 = scale_to_i16(sample->mpu2_gy_dps, 100.0f);
    packet.mpu2_rad_z_x100 = scale_to_i16(sample->mpu2_gz_dps, 100.0f);

    encode_deg_x100_to_6chars(sample->gps_lat_deg, packet.gps_lat);
    encode_deg_x100_to_6chars(sample->gps_lon_deg, packet.gps_lon);
    packet.gps_alt_m_x10 = scale_to_i16(sample->gps_altitude_m, 10.0f);
    packet.gps_sat_count_x1 = scale_to_u8(sample->gps_satellites);

    return packet;
}

esp_err_t telemetry_format_sample_csv(const flight_sample_t *sample, char *out, size_t out_len)
{
    return flight_log_format_sample_csv(sample, out, out_len);
}