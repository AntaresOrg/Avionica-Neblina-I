#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    spi_host_device_t host;
    gpio_num_t mosi_io_num;
    gpio_num_t miso_io_num;
    gpio_num_t sclk_io_num;
    gpio_num_t cs_io_num;
    int clock_speed_hz;
} flash_memory_config_t;

esp_err_t flash_memory_init(const flash_memory_config_t *cfg);

esp_err_t flash_memory_chip_erase(void);

esp_err_t flash_memory_erase_4k(uint32_t address);

esp_err_t flash_memory_read(uint32_t address, void *out, size_t len);

// Writes an arbitrary number of bytes by splitting into page programs.
// NOTE: flash must be erased beforehand (bits can only go 1->0 without erase).
esp_err_t flash_memory_write(uint32_t address, const void *data, size_t len);

// Writes up to 256 bytes (one page). Address must not cross a 256-byte boundary.
esp_err_t flash_memory_write_page(uint32_t address, const void *data, size_t len);

// Writes exactly 4096 bytes (one 4K sector) by programming 16 pages.
esp_err_t flash_memory_write_4k(uint32_t address, const void *data_4k, size_t len);

// Returns true if any byte in the 4K sector is != 0xFF.
esp_err_t flash_memory_sector_has_data_4k(uint32_t address, bool *out_has_data);

#ifdef __cplusplus
}
#endif
