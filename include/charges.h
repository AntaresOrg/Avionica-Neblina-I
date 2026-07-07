#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t reef_gpio;
    gpio_num_t chute_gpio;
} charges_config_t;

esp_err_t charges_init(const charges_config_t *cfg);

bool charges_is_initialized(void);

esp_err_t charges_set_reef(bool enabled);

esp_err_t charges_set_chute(bool enabled);

esp_err_t charges_all_off(void);

#ifdef __cplusplus
}
#endif