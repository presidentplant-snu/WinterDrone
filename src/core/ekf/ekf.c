#include "ekf.h"
#include "linalg/linalg.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

float X[STATE_DIM];       // State vector
float P[STATE_DIM][STATE_DIM];  // State covariance matrix
float Q[STATE_DIM][STATE_DIM];  // Process noise covariance
float R[MEAS_DIM][MEAS_DIM];    // Measurement noise covariance
float F[STATE_DIM][STATE_DIM];  // State transition matrix
float H[MEAS_DIM][STATE_DIM];   // Measurement matrix

void EKF_Init(void) {
    // Initialize state vector to zeros
    for (int i = 0; i < STATE_DIM; i++) {
        X[i] = 0.0f;
        for (int j = 0; j < STATE_DIM; j++) {
            P[i][j] = (i == j) ? 1.0f : 0.0f;  // Initialize P as identity matrix
            Q[i][j] = 0.0f;
            F[i][j] = (i == j) ? 1.0f : 0.0f;  // Initialize F as identity matrix
        }
    }
    
    // Set process noise covariance (tuning parameters)
    Q[0][0] = Q[1][1] = Q[2][2] = 0.001f;  // Angle process noise
    Q[3][3] = Q[4][4] = Q[5][5] = 0.003f;  // Bias process noise
    
    // Set measurement noise covariance (tuning parameters)
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            R[i][j] = (i == j) ? 10.0f : 0.0f;  // Higher values = trust accelerometer less
        }
    }
}

/**
 * Prediction step of the EKF
 */
void EKF_Predict(float gyro[3],float DT) {
    // Remove bias from gyro measurements
    float gx = gyro[0];
    float gy = gyro[1];
    float gz = gyro[2];
    float wx = gx - X[3];
    float wy = gy - X[4];
    float wz = gz - X[5];
    
    // Compute delta angles (Euler integration)
    float d_roll = wx * DT + wy * sin(X[0]) * tan(X[1]) * DT + wz * cos(X[0]) * tan(X[1]) * DT;
    float d_pitch = wy * cos(X[0]) * DT - wz * sin(X[0]) * DT;
    float d_yaw = wy * sin(X[0]) / cos(X[1]) * DT + wz * cos(X[0]) / cos(X[1]) * DT;
    
    // Update state prediction
    X[0] += d_roll;
    X[1] += d_pitch;
    X[2] += d_yaw;
    // Bias states remain constant in prediction
    
    // Jacobian of state transition function
    float J[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        J[i][i] = 1.0f;  // Diagonal elements are 1
    }
    
    // Partial derivatives for angles
    J[0][1] = (wy * cos(X[0]) * tan(X[1]) - wz * sin(X[0]) * tan(X[1])) * DT;
    J[0][2] = 0.0f;
    J[1][0] = (-wy * sin(X[0]) - wz * cos(X[0])) * DT;
    J[1][2] = 0.0f;
    J[2][0] = (wy * cos(X[0]) / cos(X[1]) - wz * sin(X[0]) / cos(X[1])) * DT;
    J[2][1] = (wy * sin(X[0]) * sin(X[1]) / (cos(X[1]) * cos(X[1])) + 
               wz * cos(X[0]) * sin(X[1]) / (cos(X[1]) * cos(X[1]))) * DT;
    
    // Effect of gyro measurements
    J[0][3] = -DT;
    J[0][4] = -sin(X[0]) * tan(X[1]) * DT;
    J[0][5] = -cos(X[0]) * tan(X[1]) * DT;
    J[1][3] = 0.0f;
    J[1][4] = -cos(X[0]) * DT;
    J[1][5] = sin(X[0]) * DT;
    J[2][3] = 0.0f;
    J[2][4] = -sin(X[0]) / cos(X[1]) * DT;
    J[2][5] = -cos(X[0]) / cos(X[1]) * DT;
    
    // Update state covariance matrix: P = J*P*J' + Q
    float JP[STATE_DIM][STATE_DIM] = {0};
    float JT[STATE_DIM][STATE_DIM] = {0};
    float JPJT[STATE_DIM][STATE_DIM] = {0};
    
    // Calculate J*P
	//
	matrix_multiply((float*)J,(float*)P,(float*)JP,STATE_DIM,STATE_DIM,STATE_DIM);
    
    // Calculate (J*P)*J^t
	matrix_transpose((float*)J,(float*)JT,STATE_DIM,STATE_DIM);
	matrix_multiply((float*)JP,(float*)JT,(float*)JPJT,STATE_DIM,STATE_DIM,STATE_DIM);
    
	matrix_add((float*)JPJT,(float*)Q,(float*)P,STATE_DIM,STATE_DIM);
}

/**
 * Update step of the EKF
 */
void EKF_Update(float accel[3], float mx, float my, float mz) {
    // Calculate the expected measurements from the current state
    float ax = accel[0];
    float ay = accel[1];
    float az = accel[2];
    float predicted_roll = X[0];
    float predicted_pitch = X[1];
    float predicted_yaw = X[2];
    
    // Calculate roll and pitch from accelerometer (simplified)
    float measured_roll = atan2(ay,az);
    float measured_pitch = atan2(-ax, sqrt(ay * ay + az * az));
    
    // TODO Calculate yaw from magnetometer (simplified)
//     float mag_x = mx * cos(measured_pitch) + mz * sin(measured_pitch);
//     float mag_y = mx * sin(measured_roll) * sin(measured_pitch) + my * cos(measured_roll) - 
                 //mz * sin(measured_roll) * cos(measured_pitch);
    //float measured_yaw = atan2(mag_y, mag_x);
    float measured_yaw = 0;
    
    // Create measurement vector
    float Z[MEAS_DIM] = {measured_roll, measured_pitch, measured_yaw};
    float h[MEAS_DIM] = {predicted_roll, predicted_pitch, predicted_yaw};
    
    // Measurement residual
    float y[MEAS_DIM];
    for (int i = 0; i < MEAS_DIM; i++) {
        y[i] = Z[i] - h[i];
    }
    // Handle wrap-around for yaw angle
    if (y[2] > M_PI) y[2] -= 2 * M_PI;
    if (y[2] < -M_PI) y[2] += 2 * M_PI;
    
    // Measurement Jacobian (H) - in this case it's a simple mapping for the angles
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            H[i][j] = 0.0f;
        }
    }
    // The first three states directly map to measurements
    H[0][0] = 1.0f;  // d(roll_measurement)/d(roll_state)
    H[1][1] = 1.0f;  // d(pitch_measurement)/d(pitch_state)
    H[2][2] = 1.0f;  // d(yaw_measurement)/d(yaw_state)
    
    // Innovation covariance: S = H*P*H' + R
    float HT[STATE_DIM][MEAS_DIM] = {0};
    float HP[MEAS_DIM][STATE_DIM] = {0};
    float HPHT[MEAS_DIM][MEAS_DIM] = {0};
    
    // Calculate H*P
	matrix_multiply((float*)H,(float*)P,(float*)HP,MEAS_DIM,STATE_DIM,STATE_DIM);

   	 
    // Calculate (HP)H^t
	matrix_transpose((float*)H,(float*)HT,MEAS_DIM,STATE_DIM);
	
	matrix_multiply((float*)HP,(float*)HT,(float*)HPHT,MEAS_DIM,STATE_DIM,MEAS_DIM);
    
    // Add measurement noise: S = HPHT + R
    float S[MEAS_DIM][MEAS_DIM];
	matrix_add((float*)HPHT,(float*)R,(float*)S,MEAS_DIM,MEAS_DIM);
    
    // Calculate Kalman gain: K = P*H'*S^(-1)
    // First, calculate S inverse (using a simplified approach for a 3x3 matrix)
    float S_inv[MEAS_DIM][MEAS_DIM];

	int ret = matrix_inverse_3x3(S,S_inv);
	if(ret == -1) return;
    
    // Calculate PH'
    float PHT[STATE_DIM][MEAS_DIM] = {0};

	matrix_multiply((float*)P,(float*)HT,(float*)PHT,STATE_DIM,STATE_DIM,MEAS_DIM);
   

    // Calculate Kalman gain = PH' * S_inv
    float K[STATE_DIM][MEAS_DIM] = {0};

	matrix_multiply((float*)PHT,(float*)S_inv,(float*)K,STATE_DIM,MEAS_DIM,MEAS_DIM);
    
    // Update state: X = X + K*y
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            X[i] += K[i][j] * y[j];
        }
    }
    
    // Update covariance: P = (I - K*H)*P
    float KH[STATE_DIM][STATE_DIM] = {0};
    float IKH[STATE_DIM][STATE_DIM];

	matrix_multiply((float*)K,(float*)H,(float*)KH,STATE_DIM,MEAS_DIM,STATE_DIM);
    
    
    // Calculate I - K*H
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    
    // Calculate (I - K*H)*P
    float IKHP[STATE_DIM][STATE_DIM] = {0};
	matrix_multiply((float*)IKH,(float*)P,(float*)IKHP,STATE_DIM,STATE_DIM,STATE_DIM);
    
    // Update P
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P[i][j] = IKHP[i][j];
        }
    }
}
