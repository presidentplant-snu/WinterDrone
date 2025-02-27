#include "sensors/mpu6050.h"
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <pico/time.h>

#include "bus/i2c.h"

// I2C Address of MPU6050
static const int addr = 0x68;

static i2c_inst_t *i2c;

static accel_scale_t accel_scale=ACCEL_16G;
static gyro_scale_t gyro_scale=GYRO_2000_DPS;
static float scaleFactors[2];

static int readMPUraw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
static void getScaleFactors(float scalefactors[2]);
static void apply_calibration(const float accel_raw[3], const float gyro_raw[3], 
                       float accel_calibrated[3], float gyro_calibrated[3]);

int initMPU(i2c_inst_t *i2c_set, accel_scale_t accel_scale_set, gyro_scale_t gyro_scale_set, DLPF_config_t dlpf_config){
	accel_scale = accel_scale_set;
	gyro_scale = gyro_scale_set;
	getScaleFactors(scaleFactors);

	i2c = i2c_set;

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

int readMPU(float accel[3], float gyro[3], float *temp){
	int ret = 0;

	int16_t rawAccel[3] = {0};
	int16_t rawGyro[3] = {0};	
	int16_t rawTemp = 0;	

	ret = readMPUraw(rawAccel,rawGyro,&rawTemp);
	if(ret == PICO_ERROR_GENERIC) return ret;
	*temp =  (float)rawTemp/340 + 36.53;

	for(int i=0; i<3; i++){
		accel[i] = rawAccel[i] / scaleFactors[0];
		gyro[i] = rawGyro[i] / scaleFactors[1];
	}

	return PICO_OK;
}

int readMPUCalibrated(float accel[3], float gyro[3], float *temp){

	float accel_raw[3] = {0}, gyro_raw[3]= {0};
	int ret = readMPU(accel_raw,gyro_raw,temp);
	if(ret == PICO_ERROR_GENERIC) return ret;
	apply_calibration(accel_raw,gyro_raw,accel,gyro);
	
	return PICO_OK;
}

static int readMPUraw(int16_t accel[3], int16_t gyro[3], int16_t *temp){
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

static void getScaleFactors(float scalefactors[2]){
	switch (accel_scale) {
		case ACCEL_2G:
			scalefactors[0] = 16384;
		case ACCEL_4G:
			scalefactors[0] = 8192;
		case ACCEL_8G:
			scalefactors[0] = 4096;
		case ACCEL_16G:
			scalefactors[0] = 2048;
		default:
			scalefactors[0]=2048;
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
		default:
			scalefactors[1]=16.4;
	}
}

// Calibration parameters
#define CALIB_SAMPLES 1000      // Number of samples to collect (adjust based on your sample rate)
#define CALIB_DISCARD 100      // Initial samples to discard (settling time)

// Calibration offsets - these will be applied to raw sensor readings
typedef struct {
    float accel_offset[3];    // X, Y, Z acceleration offsets
    float gyro_offset[3];     // X, Y, Z gyroscope offsets
} mpu_calibration_t;

// Global calibration values
static mpu_calibration_t mpu_calibration = {
    .accel_offset = {0.0f, 0.0f, 0.0f},
    .gyro_offset = {0.0f, 0.0f, 0.0f}
};

bool calibrate_mpu6050() {
    // Variables for accumulating readings
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
   	float temp = 0; 
    // Raw sensor readings
    float accel_raw[3], gyro_raw[3];
    
    // Discard initial readings to let sensors stabilize
    for (int i = 0; i < CALIB_DISCARD; i++) {
        // Read raw values from MPU6050 - replace with your actual reading function
        readMPU(accel_raw, gyro_raw,  &temp);
        sleep_ms(5);  // Small delay between readings
    }
    
    // Collect and average samples for calibration
    for (int i = 0; i < CALIB_SAMPLES; i++) {
        // Read raw values from MPU6050 - replace with your actual reading function
        readMPU(accel_raw, gyro_raw,  &temp);
        
        // Accumulate values
        for (int j = 0; j < 3; j++) {
            accel_sum[j] += accel_raw[j];
            gyro_sum[j] += gyro_raw[j];
        }
        
        sleep_ms(5);  // Small delay between readings
        
        // Show progress
        if (i % 100 == 0) {
            printf("Calibration progress: %d%%\n", (i * 100) / CALIB_SAMPLES);
        }
    }
    
    // Calculate average offsets
    for (int i = 0; i < 3; i++) {
        mpu_calibration.gyro_offset[i] = gyro_sum[i] / CALIB_SAMPLES;
        
        // For accelerometer, we only want to remove the bias, not gravity
        mpu_calibration.accel_offset[i] = accel_sum[i] / CALIB_SAMPLES;
    }
    
    // The Z-axis accelerometer should read approximately 1g (9.81 m/s²) when stationary
    // Adjust Z offset to preserve gravity
    
    mpu_calibration.accel_offset[2]-=1;

    // Print calibration values
    printf("Calibration complete!\n");
    printf("Accel offsets (X,Y,Z): %.4f, %.4f, %.4f\n", 
           mpu_calibration.accel_offset[0], 
           mpu_calibration.accel_offset[1], 
           mpu_calibration.accel_offset[2]);
    printf("Gyro offsets (X,Y,Z): %.4f, %.4f, %.4f\n", 
           mpu_calibration.gyro_offset[0], 
           mpu_calibration.gyro_offset[1], 
           mpu_calibration.gyro_offset[2]);
           
    return true;
}

static void apply_calibration(const float accel_raw[3], const float gyro_raw[3], 
                       float accel_calibrated[3], float gyro_calibrated[3]) {
    // Apply offsets to each axis
    for (int i = 0; i < 3; i++) {
        accel_calibrated[i] = accel_raw[i] - mpu_calibration.accel_offset[i];
        gyro_calibrated[i] = gyro_raw[i] - mpu_calibration.gyro_offset[i];
    }
}

