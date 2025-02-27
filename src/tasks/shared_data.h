#pragma once

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

// Data structures
typedef struct {
    bool armed;
    int8_t throttle;
    int8_t yaw;
    int8_t pitch;
    int8_t roll;
} ControlData_t;

typedef struct {
    float current_pitch;
    float current_roll;
} SensorData_t;

// Global shared data
extern volatile ControlData_t g_controlData;
extern volatile SensorData_t g_sensorData;

// Semaphores for data protection
extern SemaphoreHandle_t xControlSemaphore;
extern SemaphoreHandle_t xSensorSemaphore;

void initSemaphore();
