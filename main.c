#include <stdio.h>
#include "pico/stdlib.h"

#include "sensors/mpu6050.h"
#include "bus/i2c.h"

int main() {
	stdio_init_all();

	init_i2c(PICO_DEFAULT_I2C, PICO_DEFAULT_I2C_SDA_PIN,PICO_DEFAULT_I2C_SCL_PIN);


	printf("Hello, world!\n");

	return 0;
}
