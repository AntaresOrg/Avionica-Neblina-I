#ifndef I2C_H
#define I2C_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Protótipos das funções que seus sensores usam
void i2c_master_init(void);
esp_err_t i2c_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_write(uint8_t addr, uint8_t reg, uint8_t data);

#ifdef __cplusplus
}
#endif

#endif // I2C_H