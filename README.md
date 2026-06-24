# Aviônica do Neblina I

Este repositório contém o código da Aviônica do foguete Neblina I, da equipe Antares.

## Rodando o código

O código foi feito baseado no circuito da aviônica, onde utilizamos os seguintes componentes:

* ESP32
* 2x BMP280
* 2x MPU6050
* 1x LoRa E220 900T30D
* 1x GPS Neo-6M
* 1x Flash Memory W25Q128

Além disso o código foi escrito usando a extensão PlatformIO no VSCode, na plataforma ESPIDF. Caso queira usar o código em outras IDEs, o código relevante está em src/main.cpp

## Ligações da aviônica

### Barramento I2C

* SCL: GPIO 22
* SDA: GPIO 21
* Barramento: I2C0

### BMP280

* Endereço 1: `0x76`
* Endereço 2: `0x77`

### MPU6050

* MPU1: `0x68`
* MPU2: `0x69`

### GPS Neo-6M

* UART: UART1
* TX do GPS -> RX do ESP32: GPIO 14
* RX do GPS -> TX do ESP32: GPIO 27

### Flash W25Q128

* MOSI: GPIO 23
* MISO: GPIO 19
* SCLK: GPIO 18
* CS: GPIO 5
* Host SPI: SPI3

## Configurações compartilhadas

As definições centrais do firmware ficam em `include/avionics_config.h`.

* I2C0: GPIO 22 (SCL), GPIO 21 (SDA), 100 kHz
* GPS Neo-6M: UART1, 9600 baud, TX do GPS -> RX do ESP32 GPIO 14, RX do GPS -> TX do ESP32 GPIO 27
* Flash W25Q128: SPI3, MOSI GPIO 23, MISO GPIO 19, SCLK GPIO 18, CS GPIO 5
* Telemetria: formato CSV via LoRa
* Modo de boot atual: readback da flash (`kFlashReadbackMode = true`)
* Reset do flight log ao iniciar logging normal: `kResetFlightLogBeforeLogging`

### LoRa E220 900T30D

* A pilha de comunicação LoRa é tratada em `src/lora.c`
* O pacote de telemetria é montado em `src/telemetry.cpp`

## Status do Código

Atualmente os requisitos de código e se seu status de completion na plataforma esp-idf são:

✅ BMPs

✅ MPUs

✅ LoRa E220 900T30D

✅ GPS Neo-6M

✅ Flash Memory W25Q128

✅ Rôtina de Võo

✅ Recuperação

## Payload do LoRa

* tempo (s): short [1 : 1]
* altura relativa bmp1 (m): short [10 : 1]
* altura relativa bmp2 (m): short [10 : 1]
* aceleração linear x mpu1 (N): short [100 : 1]
* aceleração linear y mpu1 (N): short [100 : 1]
* aceleração linear z mpu1 (N): short [100 : 1]
* aceleração radial x mpu1 (N): short [100 : 1]
* aceleração radial y mpu1 (N): short [100 : 1]
* aceleração radial z mpu1 (N): short [100 : 1]
* aceleração linear x mpu2 (N): short [100 : 1]
* aceleração linear y mpu2 (N): short [100 : 1]
* aceleração linear z mpu2 (N): short [100 : 1]
* aceleração radial x mpu2 (N): short [100 : 1]
* aceleração radial y mpu2 (N): short [100 : 1]
* aceleração radial z mpu2 (N): short [100 : 1]
* latitude gps (º " '): string (6 chars)
* longitude gps (º " '): string (6 chars)
* altitude gps (m): short [10 : 1]
* quantidade satelites: char [1 : 1]

## Navegando pelo projeto

* `src/main.cpp`: ponto de entrada e orquestração geral do sistema
* `include/avionics_config.h`: configurações compartilhadas, wiring e modo de operação
* `src/bmp280.cpp` e `include/bmp280.h`: leitura e cálculo de altitude dos BMP280
* `src/mpu6050.cpp` e `include/mpu6050.h`: leitura dos acelerômetros/giroscópios MPU6050
* `src/gps.cpp` e `include/gps.h`: inicialização e leitura do GPS Neo-6M
* `src/telemetry.cpp` e `include/telemetry.h`: montagem do pacote LoRa e formatação CSV
* `src/flight_log.c` e `include/flight_log.h`: persistência e leitura dos registros na flash externa
* `src/flash_memory.cpp` e `include/flash_memory.h`: driver da flash SPI externa
* `src/lora.c` e `include/lora.h`: driver da comunicação LoRa
