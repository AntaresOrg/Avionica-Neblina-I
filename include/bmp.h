#ifndef BMP280_H
#define BMP280_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constantes de Endereçamento e Configuração
#define BMP280_ADDR_1 0x76
#define BMP280_ADDR_2 0x77
#define BMP280_CHIP_ID 0x58
#define BMP280_SEA_LEVEL_PA 101325.0f
#define BMP280_BASELINE_STABILIZATION_SAMPLES 10

/**
 * @brief Coeficientes de calibração de fábrica lidos da NVM do BMP280.
 */
typedef struct {
    uint16_t dig_T1;       /**< Termo de calibração de temperatura T1. */
    int16_t dig_T2;        /**< Termo de calibração de temperatura T2. */
    int16_t dig_T3;        /**< Termo de calibração de temperatura T3. */
    uint16_t dig_P1;       /**< Termo de calibração de pressão P1. */
    int16_t dig_P2;        /**< Termo de calibração de pressão P2. */
    int16_t dig_P3;        /**< Termo de calibração de pressão P3. */
    int16_t dig_P4;        /**< Termo de calibração de pressão P4. */
    int16_t dig_P5;        /**< Termo de calibração de pressão P5. */
    int16_t dig_P6;        /**< Termo de calibração de pressão P6. */
    int16_t dig_P7;        /**< Termo de calibração de pressão P7. */
    int16_t dig_P8;        /**< Termo de calibração de pressão P8. */
    int16_t dig_P9;        /**< Termo de calibração de pressão P9. */
} bmp280_calib_data;

/**
 * @brief Estado de execução para um sensor BMP280.
 */
typedef struct {
    uint8_t addr;                  /**< Endereço I2C (0x76 ou 0x77). */
    const char *name;              /**< Rótulo usado nos logs. */
    bmp280_calib_data calib;       /**< Coeficientes de compensação de fábrica em cache. */
    int32_t t_fine;                /**< Variável intermediária exigida pelas fórmulas do BMP280. */
    uint16_t baseline_samples;      /**< Número de amostras de altitude coletadas para a linha de base. */
    float baseline_accumulator_m;   /**< Soma corrente usada para a média da altitude da linha de base. */
    bool baseline_set;             /**< Verdadeiro quando a primeira linha de base de altitude foi capturada. */
    float baseline_altitude_m;     /**< Altitude inicial usada para saída de altitude relativa. */
    bool initialized;              /**< Verdadeiro quando o sensor é detectado e configurado. */
} bmp280_sensor_t;

// Declaração externa dos sensores globais para que outros arquivos possam acessá-los
extern bmp280_sensor_t bmp1;
extern bmp280_sensor_t bmp2;

// Protótipos das Funções Externas (Seu código chama essas funções externas)
// Certifique-se de que a função publish_line esteja declarada/visível no arquivo .cpp
extern void publish_line(const char *format, ...);

/**
 * @brief Converte pressão em Pascal para altitude em metros.
 *
 * @param pressure_pa Pressão medida em Pascal.
 * @param sea_level_pa Pressão de referência ao nível do mar em Pascal.
 * @return Altitude em metros.
 */
float bmp280_pressure_to_altitude(float pressure_pa, float sea_level_pa);

/**
 * @brief Lê os registradores de calibração de fábrica do BMP280 no estado do sensor.
 *
 * @param sensor Ponteiro para o descritor do BMP280 alvo.
 */
void bmp280_read_calibration(bmp280_sensor_t *sensor);

/**
 * @brief Valida a identidade do BMP280 usando o registrador Chip ID.
 *
 * @param sensor Ponteiro para o descritor do BMP280 alvo.
 * @return true se o Chip ID corresponder ao BMP280.
 * @return false se a leitura falhar ou o Chip ID for inesperado.
 */
bool bmp280_check_chip_id(const bmp280_sensor_t *sensor);

/**
 * @brief Inicializa um sensor BMP280.
 *
 * Verifica o Chip ID, lê os coeficientes de calibração e define o registrador
 * de controle 0xF4 para 0x27 para medições periódicas normais.
 *
 * @param sensor Ponteiro para o descritor do BMP280 alvo.
 */
void bmp280_init_sensor(bmp280_sensor_t *sensor);

/**
 * @brief Lê uma amostra do BMP280 e imprime a altitude relativa.
 *
 * Computa a pressão compensada e a altitude relativa. A primeira amostra de altitude
 * válida torna-se a linha de base (0 m).
 *
 * @param sensor Ponteiro para o descritor do BMP280 inicializado.
 */
void bmp280_read_sensor(bmp280_sensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif // BMP280_H