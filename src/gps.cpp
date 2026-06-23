#include "gps.h"

#include "neo-6m.h"

static gps_config_t g_cfg = {};

esp_err_t gps_init(const gps_config_t *cfg)
{
    if (!cfg)
        return ESP_ERR_INVALID_ARG;

    g_cfg = *cfg;
    return neo6m_setup(&g_cfg);
}

void gps_loop(void)
{
    neo6m_loop();
}

void gps_fill_sample(flight_sample_t *sample, uint32_t max_fix_age_ms)
{
    if (!sample)
        return;

    neo6m_fix_t gps_fix;
    neo6m_get_fix(&gps_fix);

    if (gps_fix.location_valid && gps_fix.location_age_ms < max_fix_age_ms)
    {
        sample->gps_lat_deg = (float)gps_fix.lat_deg;
        sample->gps_lon_deg = (float)gps_fix.lon_deg;
    }

    if (gps_fix.altitude_valid)
        sample->gps_altitude_m = gps_fix.altitude_m;

    if (gps_fix.satellites_valid)
        sample->gps_satellites = (float)gps_fix.satellites;
}