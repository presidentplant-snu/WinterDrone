#include "sensors.h"
#include "mpu.h"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"

int initI2C(){
	i2c_init(i2c_default, 400 * 1000);

	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}


int getMPU(float accel[3], float gyro[3]) {
	int a[2] = {1,2};
	
	for(int i=0; i<3; i++){
		*(accel+i)=1;
		*(gyro+i)=1;
	}
	return 0;
}
