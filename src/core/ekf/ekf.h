#pragma once 

// Constants
#define GRAVITY 9.81f
#define RAD_TO_DEG 57.295779513f
#define DEG_TO_RAD 0.017453292f
#define M_PI 3.14159265358979323846

// EKF State vector: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
#define STATE_DIM 6
#define MEAS_DIM 3 

// Function prototypes
void EKF_Init(void);
void EKF_Predict(float gyro[3],float DT);
void EKF_Update(float accel[3], float mx, float my, float mz);

// Global variables
extern float X[STATE_DIM];       // State vector
extern float P[STATE_DIM][STATE_DIM];  // State covariance matrix
extern float Q[STATE_DIM][STATE_DIM];  // Process noise covariance
extern float R[MEAS_DIM][MEAS_DIM];    // Measurement noise covariance
extern float F[STATE_DIM][STATE_DIM];  // State transition matrix
extern float H[MEAS_DIM][STATE_DIM];   // Measurement matrix
