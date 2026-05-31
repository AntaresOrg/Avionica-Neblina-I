#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// Constantes de Endereçamento e Registradores
#define MPU6050_ADDR_1         0x68
#define MPU6050_ADDR_2         0x69
#define MPU6050_WHO_AM_I_REG   0x75

/**
 * @brief Estado de execução para um sensor MPU6050.
 */
typedef struct {
    bool valid;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} mpu6050_sample_t;

typedef struct {
    uint8_t addr;          /**< Endereço I2C (0x68 ou 0x69). */
    const char *name;      /**< Rótulo usado nos logs. */
    bool initialized;      /**< Verdadeiro quando o sensor é detectado e configurado. */
} mpu6050_sensor_t;

// Declaração externa dos sensores globais alocados no arquivo .cpp
extern mpu6050_sensor_t mpu1;
extern mpu6050_sensor_t mpu2;

// Protótipos de funções do ecossistema do seu firmware (implementadas fora deste módulo)
extern void publish_line(const char *format, ...);
/**
 * @brief Inicializa um sensor MPU6050.
 *
 * Lê o registrador WHO_AM_I para validação básica de comunicação e, em seguida,
 * remove o modo de hibernação (sleep mode) escrevendo 0x00 no registrador 0x6B.
 *
 * @param sensor Ponteiro para o descritor do MPU6050 a ser inicializado.
 */
void mpu6050_init_sensor(mpu6050_sensor_t *sensor);

/**
 * @brief Lê uma amostra do MPU6050 e imprime uma linha compacta de log.
 *
 * Lê 14 bytes a partir de ACCEL_XOUT_H (0x3B), converte o acelerômetro para g
 * e o giroscópio para graus por segundo (deg/s), registrando os seis valores.
 *
 * @param sensor Ponteiro para o descritor do sensor inicializado.
 */
bool mpu6050_read_sensor(const mpu6050_sensor_t *sensor, mpu6050_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_H