#pragma once
#include "hardware/i2c.h"

typedef enum {
    ACCEL_2G  = 0b00, // ±2g
    ACCEL_4G  = 0b01, // ±4g
    ACCEL_8G  = 0b10, // ±8g
    ACCEL_16G = 0b11  // ±16g
} accel_scale_t;

typedef enum {
    GYRO_250_DPS  = 0b00, // ±250 °/s
    GYRO_500_DPS  = 0b01, // ±500 °/s
    GYRO_1000_DPS = 0b10, // ±1000 °/s
    GYRO_2000_DPS = 0b11  // ±2000 °/s
} gyro_scale_t;

/*
 * Digital Low Pass Filter (DLPF) Configuration Table (Accelerometer Sample Rate = 1kHz)
 * ---------------------------------------------------------------
 * DLPF  |     Accelerometer     |      Gyroscope       | Sample
 * CFG   | Bandwidth  |  Delay   | Bandwidth  |  Delay  | Rate
 *       |    (Hz)    |   (ms)   |    (Hz)    |   (ms)  | (kHz)
 * ---------------------------------------------------------------
 *   0   |    260     |   0.0    |    256     |   0.98  |   8
 *   1   |    184     |   2.0    |    188     |   1.9   |   1
 *   2   |     94     |   3.0    |     98     |   2.8   |   1
 *   3   |     44     |   4.9    |     42     |   4.8   |   1
 *   4   |     21     |   8.5    |     20     |   8.3   |   1
 *   5   |     10     |   13.8   |     10     |   13.4  |   1
 *   6   |      5     |   19.0   |      5     |   18.6  |   1
 *   7   |  RESERVED  | RESERVED |  RESERVED  | RESERVED |   8
 * ---------------------------------------------------------------
 */

typedef enum {
	DLPF_none = 0b000,
	DLPF_1 = 1,
	DLPF_2 = 2,
	DLPF_3 = 3,
	DLPF_4 = 4,
	DLPF_5 = 5,
	DLPF_6 = 6,
} DLPF_config_t;


int initMPU(i2c_inst_t *i2c, accel_scale_t accel_scale, gyro_scale_t gyro_scale, DLPF_config_t dlpf_config);

//static int readMPUraw(i2c_inst_t *i2c, int16_t accel[3], int16_t gyro[3], int16_t *temp);

int readMPU(i2c_inst_t *i2c, float accel[3], float gyro[3], float *temp, accel_scale_t accel_scale, gyro_scale_t gyro_scale);
