#include <stdio.h>
#include "pico/stdlib.h"

#include "sensors/mpu6050.h"
#include "bus/i2c.h"

int main() {
	stdio_init_all();

	init_i2c(PICO_DEFAULT_I2C, PICO_DEFAULT_I2C_SDA_PIN,PICO_DEFAULT_I2C_SCL_PIN);

	initMPU(PICO_DEFAULT_I2C,ACCEL_16G,GYRO_2000_DPS,DLPF_none);

    printf("Hello, world!\n");

    return 0;
}
