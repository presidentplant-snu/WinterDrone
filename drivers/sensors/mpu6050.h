#pragma once
#include "hardware/i2c.h"

// BMP280/BME280 I2C Address
#define BMP280_ADDR 0x76

/*
 * Accelerometer Scale Factors           Gyroscope Scale Factors  
 * +-----------------+------------+      +-----------------+------------+
 * | Setting         | Factor     |      | Setting         | Factor     |
 * +-----------------+------------+      +-----------------+------------+
 * | ACCEL_2G        | 16384      |      | GYRO_250_DPS    | 131        |
 * | ACCEL_4G        | 8192       |      | GYRO_500_DPS    | 65.5       |
 * | ACCEL_8G        | 4096       |      | GYRO_1000_DPS   | 32.8       |
 * | ACCEL_16G       | 2048       |      | GYRO_2000_DPS   | 2048       |
 * +-----------------+------------+      +-----------------+------------+
 */

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

typedef enum {
    BARO_ULTRA_LOW_POWER = 0b00,
    BARO_LOW_POWER       = 0b01,
    BARO_STANDARD        = 0b10,
    BARO_HIGH_RES        = 0b11,
    BARO_ULTRA_HIGH_RES  = 0b100
} baro_config_t;

int initMPU(i2c_inst_t *i2c, accel_scale_t accel_scale, gyro_scale_t gyro_scale, DLPF_config_t dlpf_config);

int readMPU(i2c_inst_t *i2c, float accel[3], float gyro[3], float *temp, accel_scale_t accel_scale, gyro_scale_t gyro_scale);

int initBarometer(i2c_inst_t *i2c);
int readBarometer(i2c_inst_t *i2c, float *pressure, float *baro_temp);

int readSensors(i2c_inst_t *i2c, float accel[3], float gyro[3], float *mpu_temp, float *pressure, float *baro_temp);
