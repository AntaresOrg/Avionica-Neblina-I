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

## Status do Código

Atualmente os requisitos de código e se seu status de completion na plataforma esp-idf são:

✅ BMPs

✅ MPUs

⚠️ LoRa E220 900T30D

⛔ GPS Neo-6M

✅ Flash Memory W25Q128

⛔ Rôtina de Võo

⛔ Recuperação

