#include "flash_memory.h"

#include <string.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MEM_FLASH";

// Flash commands
static const uint8_t CMD_READ_STATUS_1 = 0x05;
static const uint8_t CMD_WRITE_ENABLE  = 0x06;
static const uint8_t CMD_READ          = 0x03;
static const uint8_t CMD_PAGE_PROGRAM  = 0x02;
static const uint8_t CMD_SECTOR_ERASE4K = 0x20;
static const uint8_t CMD_CHIP_ERASE    = 0xC7;

static spi_device_handle_t s_dev = NULL;
static bool s_spi_inited = false;

static esp_err_t flash_transceive(const void *tx, void *rx, size_t len)
{
    if (!s_dev)
        return ESP_ERR_INVALID_STATE;
    if (!tx || len == 0)
        return ESP_ERR_INVALID_ARG;

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));

    // Full-duplex transfer. Received length equals t.length.
    t.length = len * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    return spi_device_transmit(s_dev, &t);
}

static esp_err_t flash_write_only(const void *tx, size_t len)
{
    return flash_transceive(tx, NULL, len);
}

static esp_err_t flash_read_status1(uint8_t *out_status)
{
    if (!out_status)
        return ESP_ERR_INVALID_ARG;

    // Status byte is returned while clocking one dummy byte.
    uint8_t tx[2] = {CMD_READ_STATUS_1, 0x00};
    uint8_t rx[2] = {0, 0};
    esp_err_t err = flash_transceive(tx, rx, sizeof(tx));
    if (err != ESP_OK)
        return err;

    *out_status = rx[1];
    return ESP_OK;
}

static esp_err_t flash_wait_ready(TickType_t timeout_ticks)
{
    TickType_t start = xTaskGetTickCount();
    uint8_t status = 0;

    while (true)
    {
        esp_err_t err = flash_read_status1(&status);
        if (err != ESP_OK)
            return err;

        if ((status & 0x01) == 0)
            return ESP_OK;

        if (timeout_ticks != portMAX_DELAY)
        {
            TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout_ticks)
                return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static esp_err_t flash_write_enable(void)
{
    uint8_t cmd = CMD_WRITE_ENABLE;
    return flash_write_only(&cmd, 1);
}

esp_err_t flash_memory_init(const flash_memory_config_t *cfg)
{
    if (!cfg)
        return ESP_ERR_INVALID_ARG;

    // Initialize SPI bus once.
    if (!s_spi_inited)
    {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = cfg->mosi_io_num;
        buscfg.miso_io_num = cfg->miso_io_num;
        buscfg.sclk_io_num = cfg->sclk_io_num;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = 4096 + 4;

        esp_err_t err = spi_bus_initialize(cfg->host, &buscfg, SPI_DMA_CH_AUTO);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
            return err;

        s_spi_inited = true;
    }

    if (!s_dev)
    {
        spi_device_interface_config_t devcfg = {};
        devcfg.clock_speed_hz = cfg->clock_speed_hz;
        devcfg.mode = 0;
        devcfg.spics_io_num = cfg->cs_io_num;
        devcfg.queue_size = 1;

        esp_err_t err = spi_bus_add_device(cfg->host, &devcfg, &s_dev);
        if (err != ESP_OK)
            return err;
    }

    ESP_LOGI(TAG, "SPI flash ready (host=%d, cs=%d, clk=%d Hz)", (int)cfg->host, (int)cfg->cs_io_num, cfg->clock_speed_hz);
    return ESP_OK;
}

esp_err_t flash_memory_chip_erase(void)
{
    esp_err_t err = flash_write_enable();
    if (err != ESP_OK)
        return err;

    uint8_t cmd = CMD_CHIP_ERASE;
    err = flash_write_only(&cmd, 1);
    if (err != ESP_OK)
        return err;

    return flash_wait_ready(pdMS_TO_TICKS(120000));
}

esp_err_t flash_memory_erase_4k(uint32_t address)
{
    esp_err_t err = flash_write_enable();
    if (err != ESP_OK)
        return err;

    uint8_t cmd[4] = {
        CMD_SECTOR_ERASE4K,
        (uint8_t)((address >> 16) & 0xFF),
        (uint8_t)((address >> 8) & 0xFF),
        (uint8_t)(address & 0xFF),
    };

    err = flash_write_only(cmd, sizeof(cmd));
    if (err != ESP_OK)
        return err;

    return flash_wait_ready(pdMS_TO_TICKS(5000));
}

esp_err_t flash_memory_read(uint32_t address, void *out, size_t len)
{
    if (!out || len == 0)
        return ESP_ERR_INVALID_ARG;

    // Full-duplex read: send CMD+ADDR then clock out data with dummy bytes.
    // Keep buffers bounded to avoid large stack usage.
    uint8_t *dst = (uint8_t *)out;
    size_t remaining = len;
    uint32_t addr = address;

    while (remaining > 0)
    {
        size_t chunk = remaining;
        if (chunk > 256)
            chunk = 256;

        uint8_t tx[4 + 256];
        uint8_t rx[4 + 256];
        memset(tx, 0, sizeof(tx));
        memset(rx, 0, sizeof(rx));

        tx[0] = CMD_READ;
        tx[1] = (uint8_t)((addr >> 16) & 0xFF);
        tx[2] = (uint8_t)((addr >> 8) & 0xFF);
        tx[3] = (uint8_t)(addr & 0xFF);

        esp_err_t err = flash_transceive(tx, rx, 4 + chunk);
        if (err != ESP_OK)
            return err;

        memcpy(dst, rx + 4, chunk);

        dst += chunk;
        addr += (uint32_t)chunk;
        remaining -= chunk;
    }

    return ESP_OK;
}

esp_err_t flash_memory_write(uint32_t address, const void *data, size_t len)
{
    if (!data || len == 0)
        return ESP_ERR_INVALID_ARG;

    const uint8_t *p = (const uint8_t *)data;
    size_t remaining = len;
    uint32_t addr = address;

    while (remaining > 0)
    {
        size_t page_off = addr & 0xFF;
        size_t chunk = 256 - page_off;
        if (chunk > remaining)
            chunk = remaining;

        esp_err_t err = flash_memory_write_page(addr, p, chunk);
        if (err != ESP_OK)
            return err;

        addr += (uint32_t)chunk;
        p += chunk;
        remaining -= chunk;
    }

    return ESP_OK;
}

esp_err_t flash_memory_write_page(uint32_t address, const void *data, size_t len)
{
    if (!data || len == 0 || len > 256)
        return ESP_ERR_INVALID_ARG;

    // Must not cross 256-byte page boundary.
    uint32_t page_off = address & 0xFF;
    if (page_off + len > 256)
        return ESP_ERR_INVALID_ARG;

    esp_err_t err = flash_write_enable();
    if (err != ESP_OK)
        return err;

    uint8_t header[4] = {
        CMD_PAGE_PROGRAM,
        (uint8_t)((address >> 16) & 0xFF),
        (uint8_t)((address >> 8) & 0xFF),
        (uint8_t)(address & 0xFF),
    };

    // Send header + payload in a single transaction.
    // We use a stack buffer for simplicity; for larger writes use flash_memory_write_4k.
    uint8_t frame[4 + 256];
    memcpy(frame, header, sizeof(header));
    memcpy(frame + sizeof(header), data, len);

    err = flash_write_only(frame, sizeof(header) + len);
    if (err != ESP_OK)
        return err;

    return flash_wait_ready(pdMS_TO_TICKS(50));
}

esp_err_t flash_memory_write_4k(uint32_t address, const void *data_4k, size_t len)
{
    if (!data_4k || len != 4096)
        return ESP_ERR_INVALID_ARG;

    const uint8_t *p = (const uint8_t *)data_4k;
    for (int page = 0; page < 16; page++)
    {
        uint32_t page_addr = address + (uint32_t)page * 256;
        esp_err_t err = flash_memory_write_page(page_addr, p + page * 256, 256);
        if (err != ESP_OK)
            return err;
    }

    return ESP_OK;
}

esp_err_t flash_memory_sector_has_data_4k(uint32_t address, bool *out_has_data)
{
    if (!out_has_data)
        return ESP_ERR_INVALID_ARG;

    uint8_t buf[64];
    for (uint32_t off = 0; off < 4096; off += sizeof(buf))
    {
        esp_err_t err = flash_memory_read(address + off, buf, sizeof(buf));
        if (err != ESP_OK)
            return err;

        for (size_t i = 0; i < sizeof(buf); i++)
        {
            if (buf[i] != 0xFF)
            {
                *out_has_data = true;
                return ESP_OK;
            }
        }
    }

    *out_has_data = false;
    return ESP_OK;
}
