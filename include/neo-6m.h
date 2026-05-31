#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Minimal Neo-6M (u-blox) GPS module driver for ESP-IDF.
//
// Reads NMEA sentences from an UART and parses basic fix data:
// - Location (lat/lon) from RMC/GGA
// - Satellites + altitude from GGA
//
// NOTE: In this repo, `lora.c` already uses UART_NUM_2.
// If you also use LoRa, configure GPS to use a different UART (e.g. UART_NUM_1).

typedef struct {
    uart_port_t uart_num;
    int baud_rate;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;

    // UART driver RX buffer size.
    int rx_buffer_size;

    // Consider the fix "fresh" if the last valid location update is newer than this.
    uint32_t max_fix_age_ms;
} neo6m_config_t;

typedef struct {
    bool location_valid;
    bool altitude_valid;
    bool satellites_valid;

    // Decimal degrees.
    double lat_deg;
    double lon_deg;

    // Meters above mean sea level (from GGA), when valid.
    float altitude_m;

    uint8_t satellites;

    // Age of the last valid location update.
    uint32_t location_age_ms;
} neo6m_fix_t;

// Equivalent to your Arduino `gpsSetup()`.
// Installs the UART driver and configures pins.
// If cfg==NULL, uses a conservative default config (UART1, 9600, no pins).
esp_err_t neo6m_setup(const neo6m_config_t *cfg);

// Equivalent to your Arduino `gpsLoop()`.
// Call frequently; feeds the NMEA parser with bytes read from UART.
void neo6m_loop(void);

// Returns true if the module has a valid, fresh location fix.
bool neo6m_has_fix(void);

// Copies the latest parsed fix snapshot.
// Always succeeds; fields indicate validity.
void neo6m_get_fix(neo6m_fix_t *out);

// Equivalent to your Arduino `gpsAltitude()`.
// Returns GPS altitude in meters, or NAN if unavailable.
float neo6m_altitude_m(void);

// Equivalent to your Arduino `gpsPayload()`.
// Formats either:
//   "LAT:<..> LON:<..> SAT:<..> ALT_GPS:<..>"  (ALT_GPS optional)
// or "[GPS] Sem fix | SAT:<..>".
//
// Returns ESP_OK on success, ESP_ERR_INVALID_ARG on bad args, ESP_ERR_NO_MEM if output is too small.
esp_err_t neo6m_payload(char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
