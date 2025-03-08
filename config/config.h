#include <stdint.h>

#define I2C_BAUD 400*1000

#define SSID "PicoTesting"
#define PSK "1234567890" 

#define MAX_THRUST 1.0f

#define THROTTLE_GAIN 0.9f
#define YAW_GAIN 0.1f

#define PITCH_KP 0.01 
#define PITCH_KI 0
#define PITCH_KD 0
// Max Absolute value of pid control loop
#define PITCH_MAX_OUTPUT 0.1

#define PITCH_WINDUP_EN 1
#define PITCH_WINDUP 0

#define ROLL_KP 0.01 
#define ROLL_KI 0
#define ROLL_KD 0
#define ROLL_MAX_OUTPUT 0.1

#define ROLL_WINDUP_EN 1
#define ROLL_WINDUP 0.05

#define SENSOR_DELAY 1
#define SERVER_DELAY 5
#define PID_DELAY 1

#define SEMAPHORE_DELAY_MULT 1

#define MOTOR_PINS {10,11,12,13}
