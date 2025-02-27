#include "pid_task.h"
#include <stdio.h>

#include "pico/time.h"

#include "motor/motor.h"
#include "config.h"

#include "./shared_data.h"

#include "FreeRTOS.h"
#include "task.h"

void pid_task(__unused void *params){
	initMotors(motor_pins);

	ControlData_t controlData;
	SensorData_t sensorData;

	int i=0;
	int tick = xTaskGetTickCount();
	uint16_t speeds[4] = {0,0,0,0};

	for (;;) {
		// Get latest data
		if (xSemaphoreTake(xControlSemaphore, pdMS_TO_TICKS(PID_DELAY*SEMAPHORE_DELAY_MULT)) == pdTRUE) {
			controlData = g_controlData;
			xSemaphoreGive(xControlSemaphore);
		}
		if (xSemaphoreTake(xSensorSemaphore, pdMS_TO_TICKS(PID_DELAY*SEMAPHORE_DELAY_MULT)) == pdTRUE) {
			sensorData = g_sensorData;
			xSemaphoreGive(xSensorSemaphore);
		}    	

		if(!controlData.armed){
			for(int i=0; i<4; i++){
				speeds[i]=0;
			}
			setSpeeds(speeds);
			continue;
		}

		int dt = xTaskGetTickCount() - tick;
		tick += dt;

		



		vTaskDelay(pdMS_TO_TICKS(PID_DELAY));
	}
}
