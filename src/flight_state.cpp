#include "flight_state.h"

#include <math.h>
#include <stdio.h>

static bool altitude_is_valid(float altitude_m)
{
    return isfinite(altitude_m) && !isnan(altitude_m);
}

void flight_state_controller_init(flight_state_controller_t *controller,
                                  const flight_state_thresholds_t *thresholds)
{
    if (!controller)
        return;

    controller->thresholds = thresholds ? *thresholds : flight_state_thresholds_t{};
    controller->phase = FLIGHT_STATE_GROUND;
    controller->chute_deployed = false;
    controller->reef_deployed = false;
    controller->max_altitude_m = NAN;
    controller->current_altitude_m = NAN;
}

flight_state_phase_t flight_state_controller_update(flight_state_controller_t *controller,
                                                    float current_altitude_m)
{
    if (!controller)
        return FLIGHT_STATE_GROUND;

    if (altitude_is_valid(current_altitude_m))
    {
        controller->current_altitude_m = current_altitude_m;

        if (!altitude_is_valid(controller->max_altitude_m) ||
            current_altitude_m > controller->max_altitude_m)
        {
            controller->max_altitude_m = current_altitude_m;
        }

        if (!controller->chute_deployed &&
            altitude_is_valid(controller->max_altitude_m) &&
            (controller->max_altitude_m - current_altitude_m) > controller->thresholds.altitude_offset)
            controller->chute_deployed = true;

        if (controller->chute_deployed && !controller->reef_deployed &&
            current_altitude_m < controller->thresholds.reef_altitude_m)
        {
            controller->reef_deployed = true;
        }
    }

    float resolved_altitude_m = controller->current_altitude_m;
    if (!altitude_is_valid(resolved_altitude_m))
        return controller->phase;

    if (!controller->chute_deployed && resolved_altitude_m < controller->thresholds.ground_altitude_m)
    {
        controller->phase = FLIGHT_STATE_GROUND;
    }
    else if (controller->chute_deployed && controller->reef_deployed &&
             resolved_altitude_m < controller->thresholds.ground_altitude_m)
    {
        controller->phase = FLIGHT_STATE_RECOVERY;
    }
    else if (controller->chute_deployed && !controller->reef_deployed)
    {
        controller->phase = FLIGHT_STATE_CHUTE;
    }
    else if (controller->chute_deployed && controller->reef_deployed)
    {
        controller->phase = FLIGHT_STATE_REEF;
    }
    else
    {
        controller->phase = FLIGHT_STATE_FLIGHT;
    }

    return controller->phase;
}

bool flight_state_should_log_and_save(const flight_state_controller_t *controller)
{
    if (!controller)
        return false;

    return controller->phase != FLIGHT_STATE_GROUND;
}

bool flight_state_should_send_component_status(const flight_state_controller_t *controller)
{
    if (!controller)
        return false;

    return controller->phase == FLIGHT_STATE_GROUND;
}

float flight_state_select_current_altitude(float bmp1_relative_altitude_m,
                                           float bmp2_relative_altitude_m)
{
    const bool bmp1_valid = altitude_is_valid(bmp1_relative_altitude_m);
    const bool bmp2_valid = altitude_is_valid(bmp2_relative_altitude_m);

    if (bmp1_valid && bmp2_valid)
        return (bmp1_relative_altitude_m + bmp2_relative_altitude_m) * 0.5f;
    if (bmp1_valid)
        return bmp1_relative_altitude_m;
    if (bmp2_valid)
        return bmp2_relative_altitude_m;

    return NAN;
}

const char *flight_state_name(flight_state_phase_t phase)
{
    switch (phase)
    {
        case FLIGHT_STATE_GROUND:
            return "GROUND";
        case FLIGHT_STATE_FLIGHT:
            return "FLIGHT";
        case FLIGHT_STATE_CHUTE:
            return "CHUTE";
        case FLIGHT_STATE_REEF:
            return "REEF";
        case FLIGHT_STATE_RECOVERY:
            return "RECOVERY";
        default:
            return "UNKNOWN";
    }
}

esp_err_t flight_state_format_component_status(const flight_state_controller_t *controller,
                                               const avionics_component_status_t *status,
                                               char *out,
                                               size_t out_len)
{
    if (!controller || !status || !out || out_len == 0)
        return ESP_ERR_INVALID_ARG;

    int written = snprintf(out,
                           out_len,
                           "STATE=%s,ALT=%.2f,CHUTE=%u,REEF=%u,LORA=%u,GPS=%u,BMP1=%u,BMP2=%u,MPU1=%u,MPU2=%u,FLASH=%u",
                           flight_state_name(controller->phase),
                           controller->current_altitude_m,
                           controller->chute_deployed ? 1u : 0u,
                           controller->reef_deployed ? 1u : 0u,
                           status->lora_ready ? 1u : 0u,
                           status->gps_ready ? 1u : 0u,
                           status->bmp1_ready ? 1u : 0u,
                           status->bmp2_ready ? 1u : 0u,
                           status->mpu1_ready ? 1u : 0u,
                           status->mpu2_ready ? 1u : 0u,
                           status->flash_ready ? 1u : 0u);

    if (written < 0 || (size_t)written >= out_len)
        return ESP_ERR_NO_MEM;

    return ESP_OK;
}