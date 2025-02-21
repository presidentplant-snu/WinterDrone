#include <boards/pico2_w.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"
#include "bus/i2c.h"
#include "pico/binary_info.h"
#include "drivers/sensors/bmp280.h"
#include "motor/motor.h"

int main() {
	stdio_init_all();

	init_i2c(i2c_default);
 	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
	
	initBMP(i2c_default);
	calibrateBMP(i2c_default);
	
	uint32_t pressure =0;
	int32_t temperature = 0;

	while(1){
 		int a=	readBMP(i2c_default, &pressure, &temperature);
		if(a == PICO_ERROR_GENERIC) printf("error\n");
		printf("Pressure: %f", (float)pressure/25600.0);
		//printf("Pressure:\n");
		printf("\t Temperature: %f\n",(float)temperature/100.0);
		sleep_ms(1000);
	}

	return 0;
}
