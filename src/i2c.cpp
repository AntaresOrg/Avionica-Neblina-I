#include <stdio.h>
#include <stdint.h>
#include "i2c.h"

/**
 * @brief Configure and start ESP32 I2C master peripheral.
 *
 * Uses the constants defined at the top of this file for pins, port and bus speed.
 */
void i2c_master_init(void)
{
    i2c_config_t conf = {};

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ---------- I2C ----------
/**
 * @brief Read bytes from a register in an I2C device.
 *
 * @param addr 7-bit I2C address of the target device.
 * @param reg Register address to start reading from.
 * @param data Output buffer where read bytes are stored.
 * @param len Number of bytes to read.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

/**
 * @brief Write one byte to an I2C device register.
 *
 * @param addr 7-bit I2C address of the target device.
 * @param reg Register address to write.
 * @param data Byte value to write.
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, addr, buf, 2, pdMS_TO_TICKS(100));
}

