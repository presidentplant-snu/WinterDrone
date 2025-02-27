#include <boards/pico2_w.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <hardware/i2c.h>
#include "bus/i2c.h"
#include <pico/binary_info.h>
#include "drivers/sensors/bmp280.h"
#include "drivers/sensors/mpu6050.h"
#include "drivers/wifi/wifi.h"
#include "core/ekf/ekf.h"
#include "mongoose.h"
#include "motor/motor.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>

void main_task(__unused void *params) {
	init_wifi("PicoTesting","1234567890",true);
	sleep_ms(1000);
	mongoose();
    // while(true) {
    //     // not much to do as LED is in another task, and we're using RAW (callback) lwIP API
	// 	printf("test\n");
    //     vTaskDelay(1000);
    // }
	deinit_wifi();
}

void ekf_task(__unused void *params){

	const float a = 0.5f;
	
	float pressure_avg = 0;
	float temp_avg = 0;

	uint32_t pressure = 0;
	int32_t temp = 0;

	calibrate_mpu6050();
	calibrateBMP();

	float accel[3]={0};
	float gyro[3]={0};
	float tempMPU={0};
	float DT = 10;
	int i=0;

    EKF_State ekf;
	ekf_init(&ekf, 0.005f, 0.001f, 0.1f, 0.0001f);
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

		if((i++)%100==0) {
			printf("Roll, Pitch, Yaw: %f, %f, %f\n",euler[0],euler[1],euler[2]);
			printf("Pressure, Temp: %0.2f, %0.2f\n", (float)pressure_avg,(float)temp_avg);
		}

		vTaskDelay(5);
	}
}
int main() {
	stdio_init_all();
	    const char *rtos_name;
    rtos_name = "FreeRTOS SMP";

	init_i2c(i2c_default);
	// Make the I2C pins available to picotool
 	bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

	sleep_ms(1000);
	initMPU(i2c_default,ACCEL_16G,GYRO_2000_DPS,DLPF_2);
	initBMP(i2c_default);

	sleep_ms(1000);
    printf("Starting %s on core 1:\n", rtos_name);
	
	//ekf_task((void*)0);
    xTaskCreate(main_task, "main_task", 4096, 0,configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(ekf_task, "ekf_task", 2048, 0,configMAX_PRIORITIES - 2, NULL);
	// xTaskCreate(mongoose, "mongoose", 2048, 0, configMAX_PRIORITIES - 1, NULL);

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
    while (true);


	return 0;
}
