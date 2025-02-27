#include "sensors/bmp280.h"

#include <stdio.h>

#include <pico/stdlib.h>
#include <hardware/i2c.h>
#include <pico/binary_info.h>

#include "bus/i2c.h"
#include "sensors/mpu6050.h"
#include "ekf/ekf.h"

#include "shared_data.h"

#include "config.h"

#include "FreeRTOS.h"
#include "task.h"

void sensors_init(){
	init_i2c(i2c_default);
	// Make the I2C pins available to picotool
	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	sleep_ms(1000);
	initMPU(i2c_default,ACCEL_16G,GYRO_2000_DPS,DLPF_2);
	initBMP(i2c_default);

	calibrate_mpu6050();
	calibrateBMP();
}


void sensors_task(__unused void *params){

	float accel[3]={0};
	float gyro[3]={0};
	float tempMPU={0};

	const float a = 0.5f;

	float pressure_avg = 0;
	float temp_avg = 0;

	uint32_t pressure = 0;
	int32_t temp = 0;

	EKF_State ekf;
	ekf_init(&ekf, SENSOR_DELAY/1000.0f, 0.001f, 0.1f, 0.0001f);
	while(true){
		// MPU6050 EKF code
		int ret = readMPUCalibrated(accel,gyro,&tempMPU);
		ekf_update(&ekf, gyro, accel);
		float euler[3];
		ekf_get_euler(&ekf, euler);

		for(int i=0; i<3; i++){
			euler[i] *= 180.0f/3.141592;
		}

		// BMP280 code

		readBMP(&pressure,&temp);

		pressure_avg = (a*(float)pressure / 25600.0f)+((1-a)*pressure_avg);

		temp_avg = (a*(float)temp /100.0f) + ((1-a)*temp_avg);

		if (xSemaphoreTake(xSensorSemaphore, pdMS_TO_TICKS(SENSOR_DELAY*SEMAPHORE_DELAY_MULT)) == pdTRUE) {
			g_sensorData.current_pitch = euler[1];
			g_sensorData.current_roll = euler[0];
			xSemaphoreGive(xSensorSemaphore);
		}

		vTaskDelay(pdMS_TO_TICKS(SENSOR_DELAY));
	}
}
