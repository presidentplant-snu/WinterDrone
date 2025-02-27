#pragma once
#include "hardware/i2c.h"


int initBMP(i2c_inst_t *i2c);
int calibrateBMP();
int readBMP( uint32_t *pressure, int32_t *temperature);

