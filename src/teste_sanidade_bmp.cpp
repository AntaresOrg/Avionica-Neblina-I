#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bmp.h"
#include "i2c.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

// ---------- MAIN ----------
/**
 * @brief ESP-IDF application entry point.
 *
 * Initializes I2C and all sensors, then continuously reads and logs data.
 */
extern "C" void app_main(void)
{
    i2c_master_init();
    bmp280_init_sensor(&bmp1);
    bmp280_init_sensor(&bmp2);

    while (1)
    {
        bmp280_read_sensor(&bmp1);
        bmp280_read_sensor(&bmp2);


        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}