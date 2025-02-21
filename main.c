#include <boards/pico2_w.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "bus/i2c.h"
#include "pico/binary_info.h"
#include "drivers/sensors/mpu6050.h"
#include "motor/motor.h"

int main() {
	stdio_init_all();

	init_i2c(i2c_default);

 bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	initBarometer(i2c_default,BARO_STANDARD);

	float pressure=0,baro_temp = 0;

	while(1){
 readBarometer(i2c_default,&pressure, &baro_temp);
		printf("Pressure: %f", pressure);
		//printf("Pressure:\n");
		printf("\t Temperature: %f\n",baro_temp);
		sleep_ms(1000);
	}

	return 0;
}
