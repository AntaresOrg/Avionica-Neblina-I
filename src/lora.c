#include "lora.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LORA_UART UART_NUM_2

#define LORA_TX_PIN 17
#define LORA_RX_PIN 16

// Set to 1 to enable extra LoRa UART debug logs.
#ifndef LORA_DEBUG
#define LORA_DEBUG 1
#endif

static const char *TAG = "LORA";

// Set to -1 when AUX is not connected.
#define LORA_AUX_PIN -1

#define RX_BUFFER_SIZE 512

static uint8_t rx_buffer[RX_BUFFER_SIZE];

static void wait_aux(void)
{
    if (LORA_AUX_PIN < 0)
        return;

    while (gpio_get_level(LORA_AUX_PIN) == 0)
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

bool lora_init(void)
{
    uart_config_t uart_config = {
        // Must match the LoRa module UART baud rate.
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(
        LORA_UART,
        RX_BUFFER_SIZE,
        RX_BUFFER_SIZE,
        0,
        NULL,
        0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
#if LORA_DEBUG
        ESP_LOGW(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
#endif
        return false;
    }

    err = uart_param_config(LORA_UART, &uart_config);
    if (err != ESP_OK)
    {
#if LORA_DEBUG
        ESP_LOGW(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
#endif
        return false;
    }

    err = uart_set_pin(
        LORA_UART,
        LORA_TX_PIN,
        LORA_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
#if LORA_DEBUG
        ESP_LOGW(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
#endif
        return false;
    }

    // Clear any stale bytes.
    (void)uart_flush_input(LORA_UART);

#if LORA_DEBUG
    ESP_LOGI(TAG,
             "Initialized UART%d TX=%d RX=%d Baud=%d (M0/M1 should be GND for normal mode)",
             (int)LORA_UART,
             (int)LORA_TX_PIN,
             (int)LORA_RX_PIN,
             (int)uart_config.baud_rate);
    ESP_LOGI(TAG, "Note: air rate (e.g. 19.2kbps) is LoRa RF config, not UART baud");
#endif

#if LORA_AUX_PIN >= 0
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_AUX_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };

    err = gpio_config(&io_conf);
    if (err != ESP_OK)
        return false;

    wait_aux();
#endif

    return true;
}

void lora_send(const char *msg)
{
    if (!msg)
        return;

    wait_aux();

    uart_write_bytes(
        LORA_UART,
        msg,
        strlen(msg));

#if LORA_DEBUG
    ESP_LOGD(TAG, "TX %u bytes", (unsigned)strlen(msg));
#endif

    wait_aux();
}

void lora_send_line(const char *text)
{
    if (!text)
        return;

    size_t len = strlen(text);
    if (len == 0)
        return;

    // Keep some headroom for "\r\n" and null terminator.
    if (len > 240)
        len = 240;

    char frame[256];
    int frame_len = snprintf(frame, sizeof(frame), "%.*s\r\n", (int)len, text);
    if (frame_len <= 0)
        return;

#if LORA_DEBUG
    ESP_LOGD(TAG, "TX line (%d bytes)", frame_len);
#endif

    lora_send(frame);
}

bool lora_available(void)
{
    size_t buffered = 0;

    if (uart_get_buffered_data_len(LORA_UART, &buffered) != ESP_OK)
        return false;

#if LORA_DEBUG
    if (buffered > 0)
        ESP_LOGD(TAG, "RX buffered=%u", (unsigned)buffered);
#endif

    return buffered > 0;
}

int lora_read(char *buffer, size_t max_len)
{
    if (!buffer || max_len == 0)
        return 0;

    size_t to_read = max_len - 1;
    if (to_read > RX_BUFFER_SIZE)
        to_read = RX_BUFFER_SIZE;

    int len = uart_read_bytes(
        LORA_UART,
        rx_buffer,
        to_read,
        pdMS_TO_TICKS(10));

    if (len <= 0)
        return 0;

    memcpy(buffer, rx_buffer, len);

    buffer[len] = '\0';

#if LORA_DEBUG
    ESP_LOGD(TAG, "RX %d bytes", len);
#endif

    return len;
}