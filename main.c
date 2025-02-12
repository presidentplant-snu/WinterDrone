#include <stdio.h>
#include "pico/stdlib.h"

#include "sensors/mpu6050.h"
#include "bus/i2c.h"

int main() {
	stdio_init_all();

	init_i2c(PICO_DEFAULT_I2C, PICO_DEFAULT_I2C_SDA_PIN,PICO_DEFAULT_I2C_SCL_PIN);

	initMPU(PICO_DEFAULT_I2C,ACCEL_16G,GYRO_2000_DPS,DLPF_none);

	printf("Hello, world!\n");

	float accel[3]={0};
	float gyro[3]={0};
	float temp={0};

	while(1){
		readMPU(PICO_DEFAULT_I2C,accel,gyro,&temp, ACCEL_16G,GYRO_2000_DPS);

		printf("ax:%.2f ay:%.2f az:%.2f gx:%.2f gy:%.2f gz:%.2f t:%.1f\n", 
				accel[0], accel[1], accel[2], 
				gyro[0], gyro[1], gyro[2], 
				temp);
		sleep_ms(100);  // Add small delay to not flood the console
	}

	return 0;
}
