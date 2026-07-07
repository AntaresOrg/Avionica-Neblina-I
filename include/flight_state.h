#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FLIGHT_STATE_GROUND = 0,
    FLIGHT_STATE_FLIGHT,
    FLIGHT_STATE_CHUTE,
    FLIGHT_STATE_REEF,
    FLIGHT_STATE_RECOVERY,
} flight_state_phase_t;

typedef struct {
    bool debug_flight_mode_only;
    float altitude_offset;
    float reef_altitude_m;
    float ground_altitude_m;
} flight_state_thresholds_t;

typedef struct {
    bool lora_ready;
    bool gps_ready;
    bool flash_ready;
    bool bmp1_ready;
    bool bmp2_ready;
    bool mpu1_ready;
    bool mpu2_ready;
} avionics_component_status_t;

typedef struct {
    flight_state_thresholds_t thresholds;
    flight_state_phase_t phase;
    bool chute_deployed;
    bool reef_deployed;
    float max_altitude_m;
    float current_altitude_m;
} flight_state_controller_t;

void flight_state_controller_init(flight_state_controller_t *controller,
                                  const flight_state_thresholds_t *thresholds);

flight_state_phase_t flight_state_controller_update(flight_state_controller_t *controller,
                                                    float current_altitude_m);

bool flight_state_should_log_and_save(const flight_state_controller_t *controller);

bool flight_state_should_send_component_status(const flight_state_controller_t *controller);

float flight_state_select_current_altitude(float bmp1_relative_altitude_m,
                                           float bmp2_relative_altitude_m);

const char *flight_state_name(flight_state_phase_t phase);

esp_err_t flight_state_format_component_status(const flight_state_controller_t *controller,
                                               const avionics_component_status_t *status,
                                               char *out,
                                               size_t out_len);

#ifdef __cplusplus
}
#endif