#pragma once
#include "hardware/i2c.h"


int initBMP(i2c_inst_t *i2c);
int calibrateBMP(i2c_inst_t *i2c);
int readBMP(i2c_inst_t *i2c, uint32_t *pressure, int32_t *temperature);

