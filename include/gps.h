#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "flight_log.h"
#include "neo-6m.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef neo6m_config_t gps_config_t;

esp_err_t gps_init(const gps_config_t *cfg);

void gps_loop(void);

void gps_fill_sample(flight_sample_t *sample, uint32_t max_fix_age_ms);

#ifdef __cplusplus
}
#endif