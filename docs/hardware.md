# Hardware and Wiring

## Main Components

- ESP32 DevKit V1
- 2x BMP280 pressure sensors
- 2x MPU6050 IMU sensors
- 1x GPS Neo-6M
- 1x LoRa E220 900T30D
- 1x W25Q128 external flash
- 2x charge outputs for chute and reef deployment

## Shared Bus Configuration

The shared hardware definition lives in `include/avionics_config.h`.

- I2C bus: `I2C_NUM_0`
- SCL: GPIO 22
- SDA: GPIO 21
- Bus speed: 100 kHz

## Sensor Wiring

### BMP280

- Sensor 1 address: `0x76`
- Sensor 2 address: `0x77`

### MPU6050

- MPU1 address: `0x68`
- MPU2 address: `0x69`

### GPS Neo-6M

- UART: UART1
- GPS TX -> ESP32 RX: GPIO 14
- GPS RX -> ESP32 TX: GPIO 27
- Baud rate: 9600

### Flash W25Q128

- MOSI: GPIO 23
- MISO: GPIO 19
- SCLK: GPIO 18
- CS: GPIO 5
- Host: SPI3

### Charges

- Reef charge: GPIO 26
- Chute charge: GPIO 25

## Hardware Notes

The configuration assumes one shared I2C bus for both BMP280 and MPU6050 devices, plus separate UART and SPI peripherals for GPS, LoRa, and flash.
