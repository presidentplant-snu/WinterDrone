/**
 * @file ekf.c
 * @brief Implementation of Extended Kalman Filter for orientation estimation
 */

#include "ekf.h"
#include "linalg/linalg.h"
#include <math.h>
#include <string.h>

// Constants
#define GRAVITY 9.81f
#define EPS 1e-6f

// Helper function prototypes
static void normalize_quaternion(float* q);
static void quaternion_to_euler(const float* q, float* euler);
static void quaternion_multiply(const float* q1, const float* q2, float* result);
static void quaternion_from_gyro(const float* gyro, float dt, float* q_delta);
static void compute_jacobian_F(const float* q, const float* gyro, float dt, float F[7][7]);
static void compute_jacobian_H(const float* q, float H[3][7]);
static void predict_state(EKF_State* ekf, const float* gyro);
static void update_measurement(EKF_State* ekf, const float* accel);

void ekf_init(EKF_State* ekf, float dt, float process_noise, float measurement_noise, float bias_noise) {
    // Initialize quaternion to identity rotation (w=1, x=y=z=0)
    ekf->q[0] = 1.0f;
    ekf->q[1] = 0.0f;
    ekf->q[2] = 0.0f;
    ekf->q[3] = 0.0f;
    
    // Initialize gyro bias to zero
    ekf->bias[0] = 0.0f;
    ekf->bias[1] = 0.0f;
    ekf->bias[2] = 0.0f;
    
    // Initialize error covariance matrix to identity
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < 7; i++) {
        ekf->P[i][i] = 1.0f;
    }
    
    // Initialize process noise covariance
    memset(ekf->Q, 0, sizeof(ekf->Q));
    for (int i = 0; i < 4; i++) {
        ekf->Q[i][i] = process_noise;
    }
    for (int i = 4; i < 7; i++) {
        ekf->Q[i][i] = bias_noise;
    }
    
    // Initialize measurement noise covariance
    memset(ekf->R, 0, sizeof(ekf->R));
    for (int i = 0; i < 3; i++) {
        ekf->R[i][i] = measurement_noise;
    }
    
    // Initialize time step
    ekf->dt = dt;
    
    // Set initialized flag
    ekf->initialized = 1;
}

int ekf_update(EKF_State* ekf, const float* gyro, const float* accel) {
    // Check if ekf is initialized
    if (!ekf->initialized) {
        return -1;
    }
   	 
    // Correct gyro measurements with estimated bias
    float gyro_corrected[3];
    for (int i = 0; i < 3; i++) {
        gyro_corrected[i] = (gyro[i])*(M_PI)/(180.0f) - ekf->bias[i];
    }
    
    // Prediction step
    predict_state(ekf, gyro_corrected);
   	
	float accel_corrected[3];
    for (int i = 0; i < 3; i++) {
        accel_corrected[i] = (accel[i])*(GRAVITY);
    }
    // Check if accelerometer data is valid (non-zero norm)
    float accel_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    if (fabsf(accel_norm) > EPS) {
        // Update step with accelerometer measurements
        update_measurement(ekf, accel);
    }
    
    // Normalize quaternion
    normalize_quaternion(ekf->q);
    
    return 0;
}

void ekf_get_quaternion(const EKF_State* ekf, float* q) {
    memcpy(q, ekf->q, 4 * sizeof(float));
}

void ekf_get_euler(const EKF_State* ekf, float* euler) {
    quaternion_to_euler(ekf->q, euler);
}

void ekf_get_gyro_bias(const EKF_State* ekf, float* bias) {
    memcpy(bias, ekf->bias, 3 * sizeof(float));
}

void ekf_reset(EKF_State* ekf) {
    // Reset quaternion to identity
    ekf->q[0] = 1.0f;
    ekf->q[1] = 0.0f;
    ekf->q[2] = 0.0f;
    ekf->q[3] = 0.0f;
    
    // Reset bias to zero
    ekf->bias[0] = 0.0f;
    ekf->bias[1] = 0.0f;
    ekf->bias[2] = 0.0f;
    
    // Reset error covariance to identity
    memset(ekf->P, 0, sizeof(ekf->P));
    for (int i = 0; i < 7; i++) {
        ekf->P[i][i] = 1.0f;
    }
}

// Helper function implementations

static void normalize_quaternion(float* q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > EPS) {
        q[0] /= norm;
        q[1] /= norm;
        q[2] /= norm;
        q[3] /= norm;
    } else {
        // Default to identity quaternion if normalization fails
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }
}

static void quaternion_to_euler(const float* q, float* euler) {
    // Convert quaternion to Euler angles (roll, pitch, yaw)
    // Roll (x-axis rotation)
    euler[0] = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    
    // Pitch (y-axis rotation)
    float sinp = 2.0f * (q[0] * q[2] - q[3] * q[1]);
    if (fabsf(sinp) >= 1.0f) {
        // Use 90 degrees if out of range
        euler[1] = copysignf(M_PI / 2.0f, sinp);
    } else {
        euler[1] = asinf(sinp);
    }
    
    // Yaw (z-axis rotation)
    euler[2] = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));
}

static void quaternion_multiply(const float* q1, const float* q2, float* result) {
    // Quaternion multiplication: result = q1 * q2
    result[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    result[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    result[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    result[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}

static void quaternion_from_gyro(const float* gyro, float dt, float* q_delta) {
    // Create quaternion from angular velocity (small angle approximation)
    float omega_norm = sqrtf(gyro[0]*gyro[0] + gyro[1]*gyro[1] + gyro[2]*gyro[2]);
    
    if (omega_norm < EPS) {
        // No rotation
        q_delta[0] = 1.0f;
        q_delta[1] = 0.0f;
        q_delta[2] = 0.0f;
        q_delta[3] = 0.0f;
        return;
    }
    
    float half_angle = 0.5f * omega_norm * dt;
    float sin_half_angle = sinf(half_angle);
    
    q_delta[0] = cosf(half_angle);
    q_delta[1] = (gyro[0] / omega_norm) * sin_half_angle;
    q_delta[2] = (gyro[1] / omega_norm) * sin_half_angle;
    q_delta[3] = (gyro[2] / omega_norm) * sin_half_angle;
}

static void compute_jacobian_F(const float* q, const float* gyro, float dt, float F[7][7]) {
    // State transition Jacobian matrix
    // Initialize to identity
    memset(F, 0, 7 * 7 * sizeof(float));
    for (int i = 0; i < 7; i++) {
        F[i][i] = 1.0f;
    }
    
    // Quaternion differentiation with respect to quaternion
    float wx = gyro[0] * dt * 0.5f;
    float wy = gyro[1] * dt * 0.5f;
    float wz = gyro[2] * dt * 0.5f;
    
    F[0][0] = 1.0f;
    F[0][1] = -wx;
    F[0][2] = -wy;
    F[0][3] = -wz;
    
    F[1][0] = wx;
    F[1][1] = 1.0f;
    F[1][2] = wz;
    F[1][3] = -wy;
    
    F[2][0] = wy;
    F[2][1] = -wz;
    F[2][2] = 1.0f;
    F[2][3] = wx;
    
    F[3][0] = wz;
    F[3][1] = wy;
    F[3][2] = -wx;
    F[3][3] = 1.0f;
    
    // Quaternion differentiation with respect to gyro bias
    F[0][4] = q[1] * dt * 0.5f;
    F[0][5] = q[2] * dt * 0.5f;
    F[0][6] = q[3] * dt * 0.5f;
    
    F[1][4] = -q[0] * dt * 0.5f;
    F[1][5] = q[3] * dt * 0.5f;
    F[1][6] = -q[2] * dt * 0.5f;
    
    F[2][4] = -q[3] * dt * 0.5f;
    F[2][5] = -q[0] * dt * 0.5f;
    F[2][6] = q[1] * dt * 0.5f;
    
    F[3][4] = q[2] * dt * 0.5f;
    F[3][5] = -q[1] * dt * 0.5f;
    F[3][6] = -q[0] * dt * 0.5f;
}

static void compute_jacobian_H(const float* q, float H[3][7]) {
    // Measurement Jacobian matrix (derivative of gravity vector with respect to state)
    memset(H, 0, 3 * 7 * sizeof(float));
    
    // Using quaternion rotation of gravity vector [0, 0, g]
    // Partial derivatives with respect to quaternion components
    float qw = q[0], qx = q[1], qy = q[2], qz = q[3];
    float g = GRAVITY;

    // Accelerometer x-axis
    H[0][0] = 2.0f * g * (qy * qz - qw * qx);
    H[0][1] = 2.0f * g * (qw * qy + qx * qz);
    H[0][2] = 2.0f * g * (qw * qx + qy * qz);
    H[0][3] = 2.0f * g * (qx * qy - qw * qz);
    
    // Accelerometer y-axis
    H[1][0] = 2.0f * g * (qw * qy - qx * qz);
    H[1][1] = 2.0f * g * (qw * qz - qx * qy);
    H[1][2] = 2.0f * g * (qw * qw - qx * qx - qz * qz + qy * qy);
    H[1][3] = 2.0f * g * (qy * qz - qw * qx);
    
    // Accelerometer z-axis
    H[2][0] = 2.0f * g * (qw * qz + qx * qy);
    H[2][1] = 2.0f * g * (qx * qz - qw * qy);
    H[2][2] = 2.0f * g * (qw * qx - qy * qz);
    H[2][3] = 2.0f * g * (qw * qw - qx * qx - qy * qy + qz * qz);
    
    // No effect of gyro bias on accelerometer readings
    H[0][4] = 0.0f; H[0][5] = 0.0f; H[0][6] = 0.0f;
    H[1][4] = 0.0f; H[1][5] = 0.0f; H[1][6] = 0.0f;
    H[2][4] = 0.0f; H[2][5] = 0.0f; H[2][6] = 0.0f;
}

static void predict_state(EKF_State* ekf, const float* gyro) {
    // Predict next state
    
    // Create quaternion from gyro data
    float q_delta[4];
    quaternion_from_gyro(gyro, ekf->dt, q_delta);
    
    // Update quaternion using quaternion multiplication
    float q_old[4];
    memcpy(q_old, ekf->q, 4 * sizeof(float));
    quaternion_multiply(q_old, q_delta, ekf->q);
    
    // Normalize updated quaternion
    normalize_quaternion(ekf->q);
    
    // Compute Jacobian F
    float F[7][7];
    compute_jacobian_F(q_old, gyro, ekf->dt, F);
    
    // Update covariance: P = F*P*F' + Q
    // Using linear algebra functions from linalg.h
    float F_P[7][7];
    float P_new[7][7];
    float F_transpose[7][7];
    
    // Flatten 2D arrays to 1D for using linalg functions
    float F_flat[7*7], P_flat[7*7], F_P_flat[7*7], F_transpose_flat[7*7], P_new_flat[7*7], Q_flat[7*7];
    
    // Flatten F
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            F_flat[i*7 + j] = F[i][j];
        }
    }
    
    // Flatten P
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_flat[i*7 + j] = ekf->P[i][j];
        }
    }
    
    // Flatten Q
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            Q_flat[i*7 + j] = ekf->Q[i][j];
        }
    }
    
    // Compute F*P
    matrix_multiply(F_flat, P_flat, F_P_flat, 7, 7, 7);
    
    // Compute F'
    matrix_transpose(F_flat, F_transpose_flat, 7, 7);
    
    // Compute F*P*F'
    matrix_multiply(F_P_flat, F_transpose_flat, P_new_flat, 7, 7, 7);
    
    // Add Q: P_new = F*P*F' + Q
    matrix_add(P_new_flat, Q_flat, P_new_flat, 7, 7);
    
    // Unflatten P_new to update ekf->P
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            ekf->P[i][j] = P_new_flat[i*7 + j];
        }
    }
}

static void update_measurement(EKF_State* ekf, const float* accel) {
    // Update step using accelerometer data
    
    // Normalize accelerometer data
    float accel_norm = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
    float accel_normalized[3];
    
    if (accel_norm > EPS) {
        accel_normalized[0] = accel[0] / accel_norm;
        accel_normalized[1] = accel[1] / accel_norm;
        accel_normalized[2] = accel[2] / accel_norm;
    } else {
        // Default to gravity along z-axis if normalization fails
        accel_normalized[0] = 0.0f;
        accel_normalized[1] = 0.0f;
        accel_normalized[2] = 1.0f;
    }
    
    // Compute expected gravity vector from current orientation
    float qw = ekf->q[0], qx = ekf->q[1], qy = ekf->q[2], qz = ekf->q[3];
    float expected_gravity[3];
    
    // Rotate [0, 0, 1] by quaternion
    expected_gravity[0] = 2.0f * (qx*qz - qw*qy);
    expected_gravity[1] = 2.0f * (qy*qz + qw*qx);
    expected_gravity[2] = qw*qw - qx*qx - qy*qy + qz*qz;
    
    // Compute innovation (measurement error)
    float innovation[3];
    innovation[0] = accel_normalized[0] - expected_gravity[0];
    innovation[1] = accel_normalized[1] - expected_gravity[1];
    innovation[2] = accel_normalized[2] - expected_gravity[2];
    
    // Compute measurement Jacobian
    float H[3][7];
    compute_jacobian_H(ekf->q, H);
    
    // Compute Kalman gain
    // K = P*H'*inv(H*P*H' + R)
    
    // Flatten 2D arrays for linalg functions
    float H_flat[3*7], P_flat[7*7], H_transpose_flat[7*3];
    float H_P_flat[3*7], H_P_HT_flat[3*3], R_flat[3*3];
    float S_flat[3*3], S_inv_flat[3*3], K_flat[7*3];
    
    // Flatten H
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 7; j++) {
            H_flat[i*7 + j] = H[i][j];
        }
    }
    
    // Flatten P
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            P_flat[i*7 + j] = ekf->P[i][j];
        }
    }
    
    // Flatten R
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            R_flat[i*3 + j] = ekf->R[i][j];
        }
    }
    
    // Compute H'
    matrix_transpose(H_flat, H_transpose_flat, 3, 7);
    
    // Compute H*P
    matrix_multiply(H_flat, P_flat, H_P_flat, 3, 7, 7);
    
    // Compute H*P*H'
    matrix_multiply(H_P_flat, H_transpose_flat, H_P_HT_flat, 3, 7, 3);
    
    // Compute S = H*P*H' + R
    matrix_add(H_P_HT_flat, R_flat, S_flat, 3, 3);
    
    // Compute S^-1
    if (matrix_inverse_3x3((float (*)[3])S_flat, (float (*)[3])S_inv_flat) != 0) {
        // Matrix inversion failed, skip update
        return;
    }
    
    // Compute P*H'
    float P_HT_flat[7*3];
    matrix_multiply(P_flat, H_transpose_flat, P_HT_flat, 7, 7, 3);
    
    // Compute K = P*H'*S^-1
    matrix_multiply(P_HT_flat, S_inv_flat, K_flat, 7, 3, 3);
    
    // Update state: x = x + K*y
    // For quaternion components
    float correction[7] = {0.0f};
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 3; j++) {
            correction[i] += K_flat[i*3 + j] * innovation[j];
        }
    }
    
    // Apply correction to quaternion
    ekf->q[0] += correction[0];
    ekf->q[1] += correction[1];
    ekf->q[2] += correction[2];
    ekf->q[3] += correction[3];
    normalize_quaternion(ekf->q);
    
    // Apply correction to gyro bias
    ekf->bias[0] += correction[4];
    ekf->bias[1] += correction[5];
    ekf->bias[2] += correction[6];
    
    // Update covariance: P = (I - K*H)*P
    // Compute K*H
    float KH_flat[7*7];
    memset(KH_flat, 0, 7*7*sizeof(float));
    
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 3; k++) {
                KH_flat[i*7 + j] += K_flat[i*3 + k] * H_flat[k*7 + j];
            }
        }
    }
    
    // Compute I - K*H
    float I_minus_KH_flat[7*7];
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            I_minus_KH_flat[i*7 + j] = (i == j ? 1.0f : 0.0f) - KH_flat[i*7 + j];
        }
    }
    
    // Compute (I - K*H)*P
    float P_new_flat[7*7];
    matrix_multiply(I_minus_KH_flat, P_flat, P_new_flat, 7, 7, 7);
    
    // Update P
    for (int i = 0; i < 7; i++) {
        for (int j = 0; j < 7; j++) {
            ekf->P[i][j] = P_new_flat[i*7 + j];
        }
    }
}
