#include "sensors/bmp280.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "bus/i2c.h"

// I2C Address of MPU6050
static const int addr = 0x68;
static const int baro_addr = 0x76;


