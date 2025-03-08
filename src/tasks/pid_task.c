#include "pid_task.h"
#include <stdio.h>

#include "config.h"
#include "pico/time.h"

#include "pid/pid.h"
#include "motor/motor.h"

#include "./shared_data.h"

#include "FreeRTOS.h"
#include "task.h"

void pid_task(__unused void *params){
	const uint8_t motor_pins[4] = MOTOR_PINS;

	initMotors(motor_pins);

	float speeds[4] = {0,0,0,0};

	ControlData_t controlData;
	SensorData_t sensorData;
	sensorData.current_pitch = 0.0;
	int i=0;
	int tick = xTaskGetTickCount();

    PID_Controller roll_pid;
    PID_Controller pitch_pid;


    if (!PID_Init(&roll_pid) || !PID_Init(&pitch_pid)) {
        return;
    }

 	// Configure roll PID controller
    PID_Config(&roll_pid, ROLL_KP, ROLL_KI, ROLL_KD, 0.0f, 
               -ROLL_MAX_OUTPUT, ROLL_MAX_OUTPUT);
    PID_ConfigAntiWindup(&roll_pid, ROLL_WINDUP_EN, ROLL_WINDUP);

    // Configure pitch PID controller
    PID_Config(&pitch_pid, PITCH_KP, PITCH_KI, PITCH_KD, 0.0f,
               -PITCH_MAX_OUTPUT, PITCH_MAX_OUTPUT);
    PID_ConfigAntiWindup(&pitch_pid, PITCH_WINDUP_EN, PITCH_WINDUP);

	for (;;) {
		// Get latest data
		if (xSemaphoreTake(xControlSemaphore, pdMS_TO_TICKS(PID_DELAY*SEMAPHORE_DELAY_MULT)) == pdTRUE) {
			controlData = g_controlData;
			xSemaphoreGive(xControlSemaphore);
			PID_SetSetpoint(&roll_pid,controlData.roll);
			PID_SetSetpoint(&pitch_pid,controlData.pitch);
		}
		if (xSemaphoreTake(xSensorSemaphore, pdMS_TO_TICKS(PID_DELAY*SEMAPHORE_DELAY_MULT)) == pdTRUE) {
			sensorData = g_sensorData;
			xSemaphoreGive(xSensorSemaphore);
		}    	

		if(!controlData.armed || (sensorData.current_pitch != sensorData.current_pitch)){
			for(int i=0; i<4; i++){
				speeds[i]=0.0f;
			}
			setSpeeds(speeds);
			continue;
		}

		uint32_t current_time = to_ms_since_boot(get_absolute_time());
	
		float throttle_output = THROTTLE_GAIN* (float)controlData.throttle / 100.0f ;
		float roll_output = PID_Update(&roll_pid,sensorData.current_roll,current_time,0.0);
		float pitch_output = PID_Update(&pitch_pid,-sensorData.current_pitch,current_time,0.0);
		float yaw_output = YAW_GAIN * (float)controlData.yaw / 100.0f;

	
		// From top right clockwise: 0,1,2,3
		speeds[0] = throttle_output - roll_output + pitch_output - yaw_output; 
		speeds[1] = throttle_output - roll_output - pitch_output + yaw_output;
		speeds[2] = throttle_output + roll_output - pitch_output - yaw_output;
		speeds[3] = throttle_output + roll_output + pitch_output + yaw_output;

		//printf("Outputs: %f, %f, %f, %f\n",throttle_output,roll_output,pitch_output,yaw_output);
		//printf("Speeds: %f, %f, %f, %f\n",speeds[0],speeds[1],speeds[2],speeds[3]);

		setSpeeds(speeds);
		
		vTaskDelay(pdMS_TO_TICKS(PID_DELAY));
	}
}
