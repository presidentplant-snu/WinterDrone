#include "sensors/bmp280.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "bus/i2c.h"

// I2C Address of MPU6050
static const int addr = 0x76;

static uint8_t calibrationVals[24]={0};

static int32_t bmp280_compensate_T_int32(int32_t adc_T);
static uint32_t bmp280_compensate_P_int64(int32_t adc_P);

int initBMP(i2c_inst_t *i2c){
	int ret = PICO_OK;	
	uint8_t buf = 0xB6;
	
	// Reset device
//ret = i2c_write_registers(i2c,addr,0xE0,&buf,1);
	if(ret == PICO_ERROR_GENERIC) return ret;
	sleep_ms(500); // Allow device to reset and stabilize
	
	// Set normal mode, 001(osrs_t) 011(osrs_p) 11(mode)
	buf = 0b00101111;
	ret = i2c_write_registers(i2c,addr,0xF4,&buf,1);
	if(ret == PICO_ERROR_GENERIC) return ret;

	// t_sb = 000 (0.5ms), filter = 010 (x4), 
	buf = ((0x00 << 5) | (0x02 << 2)) & 0xFC;
	ret = i2c_write_registers(i2c,addr,0xF5,&buf,1);
	if(ret == PICO_ERROR_GENERIC) return ret;

	return ret;
}

int calibrateBMP(i2c_inst_t *i2c){
	int ret =PICO_OK;
	uint8_t buf[24] = {0};

	ret = i2c_read_registers(i2c, addr, 0x88, buf, 24);
	if(ret == PICO_ERROR_GENERIC) return ret;
	for(int i=0; i<24; i++) calibrationVals[i]=buf[i];

	return ret;
}

int readBMP(i2c_inst_t *i2c, uint32_t *pressure, int32_t *temperature){
	int ret = PICO_OK;
	uint8_t buf[3] = {0};

	ret = i2c_read_registers(i2c, addr, 0xF7, buf, 3);
	if(ret == PICO_ERROR_GENERIC) return ret;
	
	*pressure = bmp280_compensate_P_int64(((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (((uint32_t)buf[2]&(0xF0)) >> 4) );

	ret = i2c_read_registers(i2c, addr, 0xFA, buf, 3);
	if(ret == PICO_ERROR_GENERIC) return ret;
	*temperature = bmp280_compensate_T_int32(((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (((int32_t)buf[2]&(0xF0)) >> 4) );

	return ret;
}


static int32_t t_fine;
// Returns temperature in DegC
static int32_t bmp280_compensate_T_int32(int32_t adc_T)
{
uint16_t dig_T1 = (calibrationVals[1] << 8) | calibrationVals[0];
int16_t dig_T2 = (calibrationVals[3] << 8) | calibrationVals[2];
int16_t dig_T3 = (calibrationVals[5] << 8) | calibrationVals[4];

int32_t var1, var2, T;
var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
((int32_t)dig_T3)) >> 14;
t_fine = var1 + var2;
T = (t_fine * 5 + 128) >> 8;
return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
uint16_t dig_P1 = (calibrationVals[7] << 8) | calibrationVals[6];
int16_t dig_P2 = (calibrationVals[9] << 8) | calibrationVals[8];
int16_t dig_P3 = (calibrationVals[11] << 8) | calibrationVals[10];
int16_t dig_P4 = (calibrationVals[13] << 8) | calibrationVals[12];
int16_t dig_P5 = (calibrationVals[15] << 8) | calibrationVals[14];
int16_t dig_P6 = (calibrationVals[17] << 8) | calibrationVals[16];
int16_t dig_P7 = (calibrationVals[19] << 8) | calibrationVals[18];
int16_t dig_P8 = (calibrationVals[21] << 8) | calibrationVals[20];
int16_t dig_P9 = (calibrationVals[23] << 8) | calibrationVals[22];

int64_t var1, var2, p;
var1 = ((int64_t)t_fine) - 128000;
var2 = var1 * var1 * (int64_t)dig_P6;
var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
var2 = var2 + (((int64_t)dig_P4)<<35);
var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
if (var1 == 0)
{
return 0; // avoid exception caused by division by zero
}
p = 1048576-adc_P;
p = (((p<<31)-var2)*3125)/var1;
var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
var2 = (((int64_t)dig_P8) * p) >> 19;
p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
return (uint32_t)p;
}
