#include "ekf.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>
#include "bus/i2c.h"
#include "sensors/mpu6050.h"

// Measurements from accelerometer (roll, pitch) and magnetometer (yaw)
/**
 * Initialize the Extended Kalman Filter
 */
i2c_inst_t *i2c=i2c_default;

void HMC5883L_Init() {
    uint8_t configA = 0x70;  // 8 samples, 15Hz output rate, normal mode
    uint8_t configB = 0xA0;  // Gain setting
    uint8_t mode    = 0x00;  // Continuous measurement mode

    i2c_write_blocking(i2c, HMC5883L_ADDR, (uint8_t[]){0x00, configA}, 2, false);
    i2c_write_blocking(i2c, HMC5883L_ADDR, (uint8_t[]){0x01, configB}, 2, false);
    i2c_write_blocking(i2c, HMC5883L_ADDR, (uint8_t[]){0x02, mode}, 2, false);
}

/**
 * Read Magnetometer Values (X, Y, Z)
 */
void HMC5883L_Read(i2c_inst_t *i2c, float *mx, float *my, float *mz) {
    uint8_t buffer[6];
    int16_t raw_x, raw_y, raw_z;

    // Read 6 bytes from magnetometer (X, Z, Y)
    int ret = i2c_read_blocking(i2c, HMC5883L_ADDR, 0x03, buffer, 6);
    if (ret == PICO_ERROR_GENERIC) {
        printf("Error reading HMC5883L!\n");
        return;
    }

    // Convert to 16-bit signed values
    raw_x = (buffer[0] << 8) | buffer[1];
    raw_z = (buffer[2] << 8) | buffer[3];
    raw_y = (buffer[4] << 8) | buffer[5];

    // Convert raw values to microteslas (µT)
    float scale = 0.92f;  // Default scale for HMC5883L
    *mx = raw_x * scale;
    *my = raw_y * scale;
    *mz = raw_z * scale;
}

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
void EKF_Predict(float gyro[3]) {
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
    float JPJT[STATE_DIM][STATE_DIM] = {0};
    
    // Calculate J*P
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                JP[i][j] += J[i][k] * P[k][j];
            }
        }
    }
    
    // Calculate (J*P)*J'
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                JPJT[i][j] += JP[i][k] * J[j][k];  // Note: using J[j][k] instead of J[k][j] for transpose
            }
        }
    }
    
    // Add process noise: P = JPJT + Q
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P[i][j] = JPJT[i][j] + Q[i][j];
        }
    }
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
    
    // Calculate yaw from magnetometer (simplified)
    float mag_x = mx * cos(measured_pitch) + mz * sin(measured_pitch);
    float mag_y = mx * sin(measured_roll) * sin(measured_pitch) + my * cos(measured_roll) - 
                 mz * sin(measured_roll) * cos(measured_pitch);
    float measured_yaw = atan2(mag_y, mag_x);
    
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
    float HP[MEAS_DIM][STATE_DIM] = {0};
    float HPHT[MEAS_DIM][MEAS_DIM] = {0};
    
    // Calculate H*P
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }
    
    // Calculate (H*P)*H'
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                HPHT[i][j] += HP[i][k] * H[j][k];  // Note: using H[j][k] instead of H[k][j] for transpose
            }
        }
    }
    
    // Add measurement noise: S = HPHT + R
    float S[MEAS_DIM][MEAS_DIM];
    for (int i = 0; i < MEAS_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            S[i][j] = HPHT[i][j] + R[i][j];
        }
    }
    
    // Calculate Kalman gain: K = P*H'*S^(-1)
    // First, calculate S inverse (using a simplified approach for a 3x3 matrix)
    float S_inv[MEAS_DIM][MEAS_DIM];
    float det = S[0][0] * (S[1][1] * S[2][2] - S[1][2] * S[2][1]) -
                S[0][1] * (S[1][0] * S[2][2] - S[1][2] * S[2][0]) +
                S[0][2] * (S[1][0] * S[2][1] - S[1][1] * S[2][0]);
    float inv_det = 1.0f / det;
    
    S_inv[0][0] = (S[1][1] * S[2][2] - S[1][2] * S[2][1]) * inv_det;
    S_inv[0][1] = (S[0][2] * S[2][1] - S[0][1] * S[2][2]) * inv_det;
    S_inv[0][2] = (S[0][1] * S[1][2] - S[0][2] * S[1][1]) * inv_det;
    S_inv[1][0] = (S[1][2] * S[2][0] - S[1][0] * S[2][2]) * inv_det;
    S_inv[1][1] = (S[0][0] * S[2][2] - S[0][2] * S[2][0]) * inv_det;
    S_inv[1][2] = (S[0][2] * S[1][0] - S[0][0] * S[1][2]) * inv_det;
    S_inv[2][0] = (S[1][0] * S[2][1] - S[1][1] * S[2][0]) * inv_det;
    S_inv[2][1] = (S[0][1] * S[2][0] - S[0][0] * S[2][1]) * inv_det;
    S_inv[2][2] = (S[0][0] * S[1][1] - S[0][1] * S[1][0]) * inv_det;
    
    // Calculate PH'
    float PHT[STATE_DIM][MEAS_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                PHT[i][j] += P[i][k] * H[j][k];  // Note: using H[j][k] instead of H[k][j] for transpose
            }
        }
    }
    
    // Calculate Kalman gain = PH' * S_inv
    float K[STATE_DIM][MEAS_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            for (int k = 0; k < MEAS_DIM; k++) {
                K[i][j] += PHT[i][k] * S_inv[k][j];
            }
        }
    }
    
    // Update state: X = X + K*y
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < MEAS_DIM; j++) {
            X[i] += K[i][j] * y[j];
        }
    }
    
    // Update covariance: P = (I - K*H)*P
    float KH[STATE_DIM][STATE_DIM] = {0};
    float IKH[STATE_DIM][STATE_DIM];
    
    // Calculate K*H
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < MEAS_DIM; k++) {
                KH[i][j] += K[i][k] * H[k][j];
            }
        }
    }
    
    // Calculate I - K*H
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            IKH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
        }
    }
    
    // Calculate (I - K*H)*P
    float IKHP[STATE_DIM][STATE_DIM] = {0};
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            for (int k = 0; k < STATE_DIM; k++) {
                IKHP[i][j] += IKH[i][k] * P[k][j];
            }
        }
    }
    
    // Update P
    for (int i = 0; i < STATE_DIM; i++) {
        for (int j = 0; j < STATE_DIM; j++) {
            P[i][j] = IKHP[i][j];
        }
    }
}
/** 
 * Multiplies two matrices: C = A * B
 * 
 * @param A Input matrix of size m x n
 * @param B Input matrix of size n x p
 * @param C Output matrix of size m x p
 * @param m Number of rows in A
 * @param n Number of columns in A / rows in B
 * @param p Number of columns in B
 */
void matrix_multiply(float *A, float *B, float *C, int m, int n, int p) {
    // Initialize C to zeros
    for (int i = 0; i < m * p; i++) {
        C[i] = 0.0f;
    }
    
    // Perform matrix multiplication
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < p; j++) {
            for (int k = 0; k < n; k++) {
                // C[i][j] += A[i][k] * B[k][j]
                C[i * p + j] += A[i * n + k] * B[k * p + j];
            }
        }
    }
}

/**
 * Computes the transpose of a matrix: AT = A^T
 * 
 * @param A Input matrix of size m x n
 * @param AT Output matrix of size n x m
 * @param m Number of rows in A
 * @param n Number of columns in A
 */
void matrix_transpose(float *A, float *AT, int m, int n) {
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {
            // AT[j][i] = A[i][j]
            AT[j * m + i] = A[i * n + j];
        }
    }
}

/**
 * Computes the inverse of a square matrix: Ainv = A^(-1)
 * This implementation uses Gaussian elimination with partial pivoting
 * and is suitable for small matrices (works best for n <= 10)
 * 
 * @param A Input square matrix of size n x n
 * @param Ainv Output matrix of size n x n
 * @param n Matrix dimension
 */
void matrix_inverse(float *A, float *Ainv, int n) {
    // Create augmented matrix [A|I]
    float *augmented = (float *)malloc(2 * n * n * sizeof(float));
    if (augmented == NULL) {
        // Handle memory allocation failure
        return;
    }
    
    // Initialize augmented matrix [A|I]
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            // Left side: copy A
            augmented[i * (2 * n) + j] = A[i * n + j];
            // Right side: identity matrix
            augmented[i * (2 * n) + (j + n)] = (i == j) ? 1.0f : 0.0f;
        }
    }
    
    // Gaussian elimination with partial pivoting
    for (int i = 0; i < n; i++) {
        // Find pivot
        int max_row = i;
        float max_val = fabsf(augmented[i * (2 * n) + i]);
        
        for (int k = i + 1; k < n; k++) {
            float val = fabsf(augmented[k * (2 * n) + i]);
            if (val > max_val) {
                max_val = val;
                max_row = k;
            }
        }
        
        // Swap rows if needed
        if (max_row != i) {
            for (int j = 0; j < 2 * n; j++) {
                float temp = augmented[i * (2 * n) + j];
                augmented[i * (2 * n) + j] = augmented[max_row * (2 * n) + j];
                augmented[max_row * (2 * n) + j] = temp;
            }
        }
        
        // Scale pivot row
        float pivot = augmented[i * (2 * n) + i];
        if (fabsf(pivot) < 1e-10) {
            // Matrix is singular or nearly singular
            free(augmented);
            // Fill result with identity as a fallback
            for (int row = 0; row < n; row++) {
                for (int col = 0; col < n; col++) {
                    Ainv[row * n + col] = (row == col) ? 1.0f : 0.0f;
                }
            }
            return;
        }
        
        for (int j = 0; j < 2 * n; j++) {
            augmented[i * (2 * n) + j] /= pivot;
        }
        
        // Eliminate other rows
        for (int k = 0; k < n; k++) {
            if (k != i) {
                float factor = augmented[k * (2 * n) + i];
                for (int j = 0; j < 2 * n; j++) {
                    augmented[k * (2 * n) + j] -= factor * augmented[i * (2 * n) + j];
                }
            }
        }
    }
    
    // Extract inverse from right side of augmented matrix
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            Ainv[i * n + j] = augmented[i * (2 * n) + (j + n)];
        }
    }
    
    free(augmented);
}




