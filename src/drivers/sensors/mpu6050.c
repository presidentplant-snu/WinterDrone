sinclude "sensors/mpu6050.h"

#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "bus/i2c.h"

// I2C Address of MPU6050
static const int addr = 0x68;


static int readMPUraw(i2c_inst_t *i2c, int16_t accel[3], int16_t gyro[3], int16_t *temp);
static void getScaleFactors(float scalefactors[2], accel_scale_t accel_scale, gyro_scale_t gyro_scale);


int initMPU(i2c_inst_t *i2c, accel_scale_t accel_scale, gyro_scale_t gyro_scale, DLPF_config_t dlpf_config){
	int ret = 0;

	// Reset device
	// Address: 0x6B (107), Power Management 1: RESET, SLEEP, CYCLE, -, TEMP_DIS, CLKSEL[2:0]
	uint8_t buf = 0x80;		
	ret = i2c_write_registers(i2c,addr,0x6B,&buf,1);
	if(ret == PICO_ERROR_GENERIC) return ret;
	sleep_ms(100); // Allow device to reset and stabilize

	// Clear sleep mode (0x6B register, 0x00 value)
	// Clear sleep mode, set clock to the interal 8 MHz Clock, enable temperature sensor    
	buf = 0x00;
	ret = i2c_write_registers(i2c,addr,0x6B,&buf,1);
	if(ret == PICO_ERROR_GENERIC) return ret;

	sleep_ms(10); // Allow stabilization after waking up

	// Set config
	//uint8_t buf2[2] = {0x6B,0x00};
    //i2c_write_blocking(i2c_default, addr, buf2, 2, false); 
	// CONFIG Register
	// Address: 0x1A, -, -, EXT_SYNC_SET[2:0], DLPF_CFG[2:0]
	// GYRO_CONFIG Register
	// Address: 0x1B, XG_ST, YG_ST, ZG_ST, AFS_SEL[1:0], -, -, -
	// ACCEL_CONFIG Register
	// Address: 0x1C, XA_ST, YA_ST, ZA_ST, AFS_SEL[1:0], -, -, -
	//
	uint8_t RA = 0x1A;
	uint8_t config[3] = {0};

	config[0] = dlpf_config;
	config[1] = gyro_scale<<3;
	config[2] = accel_scale<<3;

	ret=i2c_write_registers(i2c,addr,RA,config,3);
	if(ret == PICO_ERROR_GENERIC) return ret;
	
	return PICO_OK;
}

int readMPU(i2c_inst_t *i2c, float accel[3], float gyro[3], float *temp, 
		accel_scale_t accel_scale, gyro_scale_t gyro_scale){

	int ret = 0;

	float scaleFactors[2]={1};
	int16_t rawAccel[3] = {0};
	int16_t rawGyro[3] = {0};	
	int16_t rawTemp = 0;	

	getScaleFactors(scaleFactors,accel_scale,gyro_scale);

	ret = readMPUraw(i2c,rawAccel,rawGyro,&rawTemp);
	if(ret == PICO_ERROR_GENERIC) return ret;
	*temp =  (float)rawTemp/340 + 36.53;

	for(int i=0; i<3; i++){
		accel[i] = rawAccel[i] / scaleFactors[0];
		gyro[i] = rawGyro[i] / scaleFactors[1];
	}

	return PICO_OK;
}

static int readMPUraw(i2c_inst_t *i2c, int16_t accel[3], int16_t gyro[3], int16_t *temp){
	int ret=0;
	uint8_t buffer[14]={0};

	// Read 
	// Accel register(0x3B~0x40)
	// Temp register(0x41~0x42)
	// Gyro register (0x43~0x48)
	// All at once
	uint8_t RA = 0x3B;

	// Since we're reading right after, nostop for the address
	ret = i2c_read_registers(i2c,addr,RA,buffer,14);
	if(ret == PICO_ERROR_GENERIC) return ret;

	for (int i = 0; i < 3; i++) {
		accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
		gyro[i] = (buffer[i * 2 + 8] << 8 | buffer[((i * 2) + 1) + 8]);
	}

	*temp = buffer[6] << 8 | buffer[7];
	return PICO_OK;
}

static void getScaleFactors(float scalefactors[2], accel_scale_t accel_scale, gyro_scale_t gyro_scale){
	switch (accel_scale) {
		case ACCEL_2G:
			scalefactors[0] = 16384;
		case ACCEL_4G:
			scalefactors[0] = 8192;
		case ACCEL_8G:
			scalefactors[0] = 4096;
		case ACCEL_16G:
			scalefactors[0] = 2048;
	}
	switch (gyro_scale) {
		case GYRO_250_DPS:
			scalefactors[1] = 131;
		case GYRO_500_DPS:
			scalefactors[1] = 65.5;
		case GYRO_1000_DPS:
			scalefactors[1] = 32.8;
		case GYRO_2000_DPS:
			scalefactors[1] = 16.4;
	}
}

