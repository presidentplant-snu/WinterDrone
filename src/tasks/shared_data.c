#include "shared_data.h"

// Initialize global variables
volatile ControlData_t g_controlData = {0};
volatile SensorData_t g_sensorData = {0};
SemaphoreHandle_t xControlSemaphore = 0;
SemaphoreHandle_t xSensorSemaphore = 0;

void initSemaphore(){
	xControlSemaphore = xSemaphoreCreateMutex();
	xSensorSemaphore = xSemaphoreCreateMutex();
}
