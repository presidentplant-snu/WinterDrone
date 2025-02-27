#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>

#include "bus/i2c.h"

int init_i2c(i2c_inst_t *i2c)
{   
	i2c_init(i2c, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
	// Make the I2C pins available to picotool
}

int i2c_write_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t RA, uint8_t *buf, size_t len){
	int ret = 0;
	ret = i2c_write_blocking(i2c, addr, &RA, 1, true);
	if(ret == PICO_ERROR_GENERIC) return ret;
	ret = i2c_write_blocking(i2c, addr, buf, len, false);
	if(ret == PICO_ERROR_GENERIC) return ret;
	
	return PICO_OK;
}

int i2c_read_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t RA, uint8_t *buf, size_t len){

	int ret = 0;
	ret = i2c_write_blocking(i2c, addr, &RA, 1, true);
	if(ret == PICO_ERROR_GENERIC) return ret;
	ret = i2c_read_blocking(i2c, addr, buf, len, false);
	if(ret == PICO_ERROR_GENERIC) return ret;
	
	return PICO_OK;
}
