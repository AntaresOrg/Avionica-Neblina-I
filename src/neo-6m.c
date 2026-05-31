#include "neo-6m.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS";


#ifndef NEO6M_DEFAULT_UART
#define NEO6M_DEFAULT_UART UART_NUM_1
#endif

#ifndef NEO6M_DEFAULT_BAUD
#define NEO6M_DEFAULT_BAUD 9600
#endif

#ifndef NEO6M_DEFAULT_RX_BUF
#define NEO6M_DEFAULT_RX_BUF 2048
#endif

#ifndef NEO6M_DEFAULT_MAX_FIX_AGE_MS
#define NEO6M_DEFAULT_MAX_FIX_AGE_MS 2000
#endif

#define NEO6M_NMEA_MAX_LINE 128

typedef struct {
    neo6m_config_t cfg;
    bool initialized;

    uint32_t total_rx_bytes;
    int64_t last_rx_us;
    int64_t start_us;
    int64_t last_no_data_warn_us;

    // Most recent parsed data.
    bool location_valid;
    bool altitude_valid;
    bool satellites_valid;

    double lat_deg;
    double lon_deg;

    float altitude_m;
    uint8_t satellites;

    int64_t last_location_update_us;
    int64_t last_altitude_update_us;

    // Line assembly.
    char line[NEO6M_NMEA_MAX_LINE];
    size_t line_len;
} neo6m_state_t;

static neo6m_state_t s_gps;

static uint32_t age_ms_from_us(int64_t last_us)
{
    if (last_us <= 0) return UINT32_MAX;
    int64_t now_us = esp_timer_get_time();
    int64_t delta_us = now_us - last_us;
    if (delta_us <= 0) return 0;
    if (delta_us > (int64_t)UINT32_MAX * 1000) return UINT32_MAX;
    return (uint32_t)(delta_us / 1000);
}

static bool parse_degmin_to_deg(const char *s, bool is_lon, double *out_deg)
{
    if (!s || !*s || !out_deg) return false;

    char *end = NULL;
    double v = strtod(s, &end);
    if (end == s) return false;

    // Latitude: ddmm.mmmm, Longitude: dddmm.mmmm
    double deg_div = is_lon ? 100.0 : 100.0;
    (void)deg_div;

    double deg = floor(v / 100.0);
    double minutes = v - (deg * 100.0);
    if (deg < 0.0 || minutes < 0.0) return false;

    *out_deg = deg + (minutes / 60.0);
    return true;
}

static bool starts_with_any(const char *s, const char *a, const char *b)
{
    if (!s) return false;
    if (a && strncmp(s, a, strlen(a)) == 0) return true;
    if (b && strncmp(s, b, strlen(b)) == 0) return true;
    return false;
}

static void update_location(double lat_deg, double lon_deg)
{
    s_gps.lat_deg = lat_deg;
    s_gps.lon_deg = lon_deg;
    s_gps.location_valid = true;
    s_gps.last_location_update_us = esp_timer_get_time();

    ESP_LOGD(TAG, "FIX lat=%.6f lon=%.6f", lat_deg, lon_deg);
}

static void update_altitude(float alt_m)
{
    s_gps.altitude_m = alt_m;
    s_gps.altitude_valid = true;
    s_gps.last_altitude_update_us = esp_timer_get_time();
}

static void update_satellites(uint8_t sats)
{
    // Typical ranges are small (0..12+). Values like 99 are usually a parse artifact
    // (e.g., reading HDOP=99.99 when the sentence is missing fields).
    if (sats > 64) return;
    s_gps.satellites = sats;
    s_gps.satellites_valid = true;
    ESP_LOGD(TAG, "Satellites: %u", sats);
}

static void strip_checksum(char *line)
{
    // Replace '*' with '\0' so we can split by commas safely.
    if (!line) return;
    char *star = strchr(line, '*');
    if (star) *star = '\0';
}

static void parse_gga(char *line)
{
    // $GPGGA,hhmmss.sss,lat,NS,lon,EW,fixq,numsat,hdop,alt,M,...
    // Fields we care: 2..5, 6, 7, 9

    strip_checksum(line);

    char *fields[16] = {0};
    int n = 0;

    for (char *p = line; p && n < (int)(sizeof(fields) / sizeof(fields[0])); ) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }

    if (n < 10) return;

    const char *lat_s = fields[2];
    const char *lat_h = fields[3];
    const char *lon_s = fields[4];
    const char *lon_h = fields[5];
    const char *fixq_s = fields[6];
    const char *sats_s = fields[7];
    const char *alt_s = fields[9];

    if (!fixq_s || fixq_s[0] == '\0') return;
    int fixq = atoi(fixq_s);
    if (fixq <= 0) {
        // No fix. Still allow satellites update.
        if (sats_s && *sats_s) {
            int sats = atoi(sats_s);
            if (sats >= 0 && sats <= 255) update_satellites((uint8_t)sats);
        }
        return;
    }

    if (sats_s && *sats_s) {
        int sats = atoi(sats_s);
        if (sats >= 0 && sats <= 255) update_satellites((uint8_t)sats);
    }

    double lat = 0.0;
    double lon = 0.0;

    if (lat_s && *lat_s && lat_h && *lat_h && lon_s && *lon_s && lon_h && *lon_h) {
        if (parse_degmin_to_deg(lat_s, false, &lat) && parse_degmin_to_deg(lon_s, true, &lon)) {
            if (lat_h[0] == 'S' || lat_h[0] == 's') lat = -lat;
            if (lon_h[0] == 'W' || lon_h[0] == 'w') lon = -lon;
            update_location(lat, lon);
        }
    }

    if (alt_s && *alt_s) {
        char *end = NULL;
        double alt_d = strtod(alt_s, &end);
        if (end != alt_s) {
            update_altitude((float)alt_d);
        }
    }
}

static void parse_rmc(char *line)
{
    // $GPRMC,hhmmss.sss,status,lat,NS,lon,EW,speed,course,date,...
    // Fields we care: 2, 3..6

    strip_checksum(line);

    char *fields[16] = {0};
    int n = 0;

    for (char *p = line; p && n < (int)(sizeof(fields) / sizeof(fields[0])); ) {
        fields[n++] = p;
        char *comma = strchr(p, ',');
        if (!comma) break;
        *comma = '\0';
        p = comma + 1;
    }

    if (n < 7) return;

    const char *status = fields[2];
    if (!status || status[0] == '\0') return;

    // 'A' = active, 'V' = void
    if (status[0] != 'A' && status[0] != 'a') return;

    const char *lat_s = fields[3];
    const char *lat_h = fields[4];
    const char *lon_s = fields[5];
    const char *lon_h = fields[6];

    double lat = 0.0;
    double lon = 0.0;

    if (!lat_s || !*lat_s || !lat_h || !*lat_h || !lon_s || !*lon_s || !lon_h || !*lon_h) return;

    if (!parse_degmin_to_deg(lat_s, false, &lat)) return;
    if (!parse_degmin_to_deg(lon_s, true, &lon)) return;

    if (lat_h[0] == 'S' || lat_h[0] == 's') lat = -lat;
    if (lon_h[0] == 'W' || lon_h[0] == 'w') lon = -lon;

    update_location(lat, lon);
}

static void parse_nmea_sentence(char *line)
{
    if (!line || line[0] != '$') return;

    // Accept both GP (GPS) and GN (combined) talkers.
    if (starts_with_any(line, "$GPGGA", "$GNGGA"))
    {
        ESP_LOGD(TAG, "GGA detected");
        parse_gga(line);
    }
    else if (starts_with_any(line, "$GPRMC", "$GNRMC"))
    {
        ESP_LOGD(TAG, "RMC detected");
        parse_rmc(line);
    }
}

static void feed_char(char c)
{
    if (c == '\r') return;

    if (c == '\n') {
        if (s_gps.line_len == 0) return;
        s_gps.line[s_gps.line_len] = '\0';
        ESP_LOGD(TAG, "NMEA: %s", s_gps.line);
        parse_nmea_sentence(s_gps.line);
        s_gps.line_len = 0;
        return;
    }

    if (s_gps.line_len + 1 >= sizeof(s_gps.line)) {
        // Overflow: drop the current line.
        s_gps.line_len = 0;
        return;
    }

    s_gps.line[s_gps.line_len++] = c;
}

esp_err_t neo6m_setup(const neo6m_config_t *cfg)
{
    neo6m_config_t c = {
        .uart_num = NEO6M_DEFAULT_UART,
        .baud_rate = NEO6M_DEFAULT_BAUD,
        .tx_pin = UART_PIN_NO_CHANGE,
        .rx_pin = UART_PIN_NO_CHANGE,
        .rx_buffer_size = NEO6M_DEFAULT_RX_BUF,
        .max_fix_age_ms = NEO6M_DEFAULT_MAX_FIX_AGE_MS,
    };

    if (cfg) {
        c = *cfg;
        if (c.rx_buffer_size <= 0) c.rx_buffer_size = NEO6M_DEFAULT_RX_BUF;
        if (c.baud_rate <= 0) c.baud_rate = NEO6M_DEFAULT_BAUD;
        if (c.max_fix_age_ms == 0) c.max_fix_age_ms = NEO6M_DEFAULT_MAX_FIX_AGE_MS;
    }

    uart_config_t uart_config = {
        .baud_rate = c.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver (RX only is enough, but TX is kept enabled for flexibility).
    esp_err_t err = uart_driver_install(c.uart_num, c.rx_buffer_size, 0, 0, NULL, 0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;

    err = uart_param_config(c.uart_num, &uart_config);
    if (err != ESP_OK) return err;

    err = uart_set_pin(c.uart_num, c.tx_pin, c.rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    // Initialize state.
    memset(&s_gps, 0, sizeof(s_gps));
    s_gps.cfg = c;
    s_gps.initialized = true;
    s_gps.line_len = 0;
    s_gps.last_location_update_us = 0;
    s_gps.last_altitude_update_us = 0;
    s_gps.total_rx_bytes = 0;
    s_gps.last_rx_us = 0;
    s_gps.start_us = esp_timer_get_time();
    s_gps.last_no_data_warn_us = 0;

    ESP_LOGI(TAG,
            "Initialized UART%d TX=%d RX=%d Baud=%d",
            c.uart_num,
            c.tx_pin,
            c.rx_pin,
            c.baud_rate);

    return ESP_OK;
}

void neo6m_loop(void)
{
    if (!s_gps.initialized) return;

    uint8_t buf[128];

    while (true) {
        int n = uart_read_bytes(
            s_gps.cfg.uart_num,
            buf,
            sizeof(buf),
            pdMS_TO_TICKS(10));

        if (n > 0)
        {
            ESP_LOGD(TAG, "Received %d bytes", n);
            s_gps.total_rx_bytes += (uint32_t)n;
            s_gps.last_rx_us = esp_timer_get_time();
        }
        if (n <= 0) break;

        for (int i = 0; i < n; i++) {
            feed_char((char)buf[i]);
        }
    }

    // If we haven't received any bytes after a short grace period, warn.
    // This is the most common reason for GPS fields staying NAN.
    int64_t now_us = esp_timer_get_time();
    if (s_gps.total_rx_bytes == 0)
    {
        const int64_t grace_us = 3 * 1000 * 1000; // 3s
        const int64_t warn_period_us = 10 * 1000 * 1000; // 10s

        if ((now_us - s_gps.start_us) > grace_us &&
            (s_gps.last_no_data_warn_us == 0 || (now_us - s_gps.last_no_data_warn_us) > warn_period_us))
        {
            s_gps.last_no_data_warn_us = now_us;
            ESP_LOGW(TAG,
                     "No UART data received yet (UART%d TX=%d RX=%d baud=%d). Check wiring: GPS TX -> ESP RX (GPIO%d) and common GND; verify baud rate.",
                     (int)s_gps.cfg.uart_num,
                     (int)s_gps.cfg.tx_pin,
                     (int)s_gps.cfg.rx_pin,
                     (int)s_gps.cfg.baud_rate,
                     (int)s_gps.cfg.rx_pin);
        }
    }
}

bool neo6m_has_fix(void)
{
    if (!s_gps.location_valid) return false;

    uint32_t age = age_ms_from_us(s_gps.last_location_update_us);
    return age <= s_gps.cfg.max_fix_age_ms;
}

void neo6m_get_fix(neo6m_fix_t *out)
{
    if (!out) return;

    out->location_valid = s_gps.location_valid;
    out->altitude_valid = s_gps.altitude_valid;
    out->satellites_valid = s_gps.satellites_valid;

    out->lat_deg = s_gps.lat_deg;
    out->lon_deg = s_gps.lon_deg;

    out->altitude_m = s_gps.altitude_m;
    out->satellites = s_gps.satellites;

    out->location_age_ms = age_ms_from_us(s_gps.last_location_update_us);
}

float neo6m_altitude_m(void)
{
    if (!s_gps.altitude_valid) return NAN;
    return s_gps.altitude_m;
}

esp_err_t neo6m_payload(char *out, size_t out_len)
{
    if (!out || out_len == 0) return ESP_ERR_INVALID_ARG;

    neo6m_fix_t fix;
    neo6m_get_fix(&fix);

    if (fix.location_valid && fix.location_age_ms < s_gps.cfg.max_fix_age_ms) {
        // Similar to Arduino code: LAT/LON/SAT and optional ALT_GPS.
        int written = 0;
        if (fix.altitude_valid) {
            written = snprintf(out,
                               out_len,
                               "LAT:%.6f LON:%.6f SAT:%u ALT_GPS:%.1f",
                               fix.lat_deg,
                               fix.lon_deg,
                               (unsigned)fix.satellites,
                               (double)fix.altitude_m);
        } else {
            written = snprintf(out,
                               out_len,
                               "LAT:%.6f LON:%.6f SAT:%u",
                               fix.lat_deg,
                               fix.lon_deg,
                               (unsigned)fix.satellites);
        }

        if (written < 0) return ESP_FAIL;
        if ((size_t)written >= out_len) return ESP_ERR_NO_MEM;
        return ESP_OK;
    }

    // No fix (or stale fix): match the status text you used.
    if (fix.satellites_valid) {
        int written = snprintf(out, out_len, "[GPS] Sem fix | SAT:%u", (unsigned)fix.satellites);
        if (written < 0) return ESP_FAIL;
        if ((size_t)written >= out_len) return ESP_ERR_NO_MEM;
        return ESP_OK;
    }

    int written = snprintf(out, out_len, "[GPS] Sem fix | SAT:aguardando...");
    if (written < 0) return ESP_FAIL;
    if ((size_t)written >= out_len) return ESP_ERR_NO_MEM;
    return ESP_OK;
}
