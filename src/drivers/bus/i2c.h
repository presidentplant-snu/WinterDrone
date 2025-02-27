#pragma once
#include "pico/stdlib.h"
#include "hardware/i2c.h"

int init_i2c(i2c_inst_t *i2c);
int i2c_write_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
int i2c_read_registers(i2c_inst_t *i2c, uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
