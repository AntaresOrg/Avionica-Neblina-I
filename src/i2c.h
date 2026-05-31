#ifndef I2C_MASTER_H
#define I2C_MASTER_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

#define I2C_MASTER_SCL_IO ((gpio_num_t)22)
#define I2C_MASTER_SDA_IO ((gpio_num_t)21)
#define I2C_MASTER_NUM    I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000

/**
 * @brief Configure and start ESP32 I2C master peripheral.
 *
 * Uses the constants defined at the top of this file for pins, port and bus speed.
 */
void i2c_master_init(void);

/**
 * @brief Read bytes from a register in an I2C device.
 *
 * @param addr 7-bit I2C address of the target device.
 * @param reg Register address to start reading from.
 * @param data Output buffer where read bytes are stored.
 * @param len Number of bytes to read.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);

/**
 * @brief Write one byte to an I2C device register.
 *
 * @param addr 7-bit I2C address of the target device.
 * @param reg Register address to write.
 * @param data Byte value to write.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif // I2C_MASTER_H