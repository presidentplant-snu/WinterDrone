/**
 * 
 * @file ekf.h
 * @brief Extended Kalman Filter for orientation estimation using IMU data
 */

#pragma once 

/**
 * @brief EKF state structure
 */
typedef struct {
    float q[4];          // Quaternion (w, x, y, z)
    float bias[3];       // Gyroscope bias (x, y, z)
    float P[7][7];       // Error covariance matrix
    float Q[7][7];       // Process noise covariance
    float R[3][3];       // Measurement noise covariance
    float dt;            // Time step in seconds
    int initialized;     // Flag to check if EKF is initialized
} EKF_State;

/**
 * @brief Initialize the EKF
 * 
 * @param ekf Pointer to the EKF state structure
 * @param dt Time step in seconds
 * @param process_noise Process noise strength (gyro variance)
 * @param measurement_noise Measurement noise strength (accel variance)
 * @param bias_noise Gyroscope bias noise strength
 */
void ekf_init(EKF_State* ekf, float dt, float process_noise, float measurement_noise, float bias_noise);

/**
 * @brief Update the EKF with new sensor data
 * 
 * @param ekf Pointer to the EKF state structure
 * @param gyro 3-axis gyroscope measurements in rad/s (x, y, z)
 * @param accel 3-axis accelerometer measurements in m/s^2 (x, y, z)
 * @return int 0 if successful, non-zero otherwise
 */
int ekf_update(EKF_State* ekf, const float* gyro, const float* accel);

/**
 * @brief Get the current orientation quaternion
 * 
 * @param ekf Pointer to the EKF state structure
 * @param q Output quaternion (w, x, y, z)
 */
void ekf_get_quaternion(const EKF_State* ekf, float* q);

/**
 * @brief Get the current Euler angles (roll, pitch, yaw) in radians
 * 
 * @param ekf Pointer to the EKF state structure
 * @param euler Output Euler angles (roll, pitch, yaw) in radians
 */
void ekf_get_euler(const EKF_State* ekf, float* euler);

/**
 * @brief Get the current gyroscope bias estimates
 * 
 * @param ekf Pointer to the EKF state structure
 * @param bias Output gyroscope bias (x, y, z)
 */
void ekf_get_gyro_bias(const EKF_State* ekf, float* bias);

/**
 * @brief Reset the EKF state
 * 
 * @param ekf Pointer to the EKF state structure
 */
void ekf_reset(EKF_State* ekf);
