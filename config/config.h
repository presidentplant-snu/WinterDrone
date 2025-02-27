#include <stdint.h>

#pragma once

#define I2C_BAUD 400*1000

#define SSID "PicoTesting"
#define PSK "1234567890" 

#define PITCH_KP 1
#define PITCH_KI 0
#define PITCH_KD 0

#define ROLL_KP 1 
#define ROLL_KI 0
#define ROLL_KD 0

#define SENSOR_DELAY 1
#define SERVER_DELAY 5
#define PID_DELAY 1

#define SEMAPHORE_DELAY_MULT 1

uint8_t motor_pins[4] = {10,11,12,13};
