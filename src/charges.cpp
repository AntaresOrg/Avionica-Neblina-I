#include "charges.h"

#include <stdint.h>

static charges_config_t s_cfg = {
    .reef_gpio = GPIO_NUM_NC,
    .chute_gpio = GPIO_NUM_NC,
};
static bool s_initialized = false;

static bool charges_gpio_is_valid(gpio_num_t pin)
{
    return pin != GPIO_NUM_NC && GPIO_IS_VALID_OUTPUT_GPIO(pin);
}

static esp_err_t charges_configure_output(gpio_num_t pin)
{
    gpio_config_t io_conf = {};
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
        return err;

    return gpio_set_level(pin, 0);
}

static esp_err_t charges_set_pin(gpio_num_t pin, bool enabled)
{
    if (!s_initialized)
        return ESP_ERR_INVALID_STATE;
    if (!charges_gpio_is_valid(pin))
        return ESP_ERR_INVALID_ARG;

    return gpio_set_level(pin, enabled ? 1 : 0);
}

esp_err_t charges_init(const charges_config_t *cfg)
{
    if (!cfg)
        return ESP_ERR_INVALID_ARG;
    if (!charges_gpio_is_valid(cfg->reef_gpio) || !charges_gpio_is_valid(cfg->chute_gpio))
        return ESP_ERR_INVALID_ARG;
    if (cfg->reef_gpio == cfg->chute_gpio)
        return ESP_ERR_INVALID_ARG;

    esp_err_t reef_err = charges_configure_output(cfg->reef_gpio);
    if (reef_err != ESP_OK)
        return reef_err;

    esp_err_t chute_err = charges_configure_output(cfg->chute_gpio);
    if (chute_err != ESP_OK)
        return chute_err;

    s_cfg = *cfg;
    s_initialized = true;
    return ESP_OK;
}

bool charges_is_initialized(void)
{
    return s_initialized;
}

esp_err_t charges_set_reef(bool enabled)
{
    return charges_set_pin(s_cfg.reef_gpio, enabled);
}

esp_err_t charges_set_chute(bool enabled)
{
    return charges_set_pin(s_cfg.chute_gpio, enabled);
}

esp_err_t charges_all_off(void)
{
    if (!s_initialized)
        return ESP_ERR_INVALID_STATE;

    esp_err_t reef_err = gpio_set_level(s_cfg.reef_gpio, 0);
    if (reef_err != ESP_OK)
        return reef_err;

    return gpio_set_level(s_cfg.chute_gpio, 0);
}