// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define HMC5883L_ADDR 0x1E  // Default I2C address of HMC5883L


// Constants
#define GRAVITY 9.81f
#define RAD_TO_DEG 57.295779513f
#define DEG_TO_RAD 0.017453292f
#define DT 0.01f  // 10ms sample rate
#define M_PI 3.14159265358979323846

// EKF State vector: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
#define STATE_DIM 6
#define MEAS_DIM 3 

// Function prototypes
void EKF_Init(void);
void EKF_Predict(float gyro[3]);
void EKF_Update(float accel[3], float mx, float my, float mz);
void matrix_multiply(float *A, float *B, float *C, int m, int n, int p);
void matrix_transpose(float *A, float *AT, int m, int n);
void matrix_inverse(float *A, float *Ainv, int n);
void HMC5883L_Init();
void HMC5883L_Read(i2c_inst_t *i2c, float *mx, float *my, float *mz);

// Global variables
float X[STATE_DIM];       // State vector
float P[STATE_DIM][STATE_DIM];  // State covariance matrix
float Q[STATE_DIM][STATE_DIM];  // Process noise covariance
float R[MEAS_DIM][MEAS_DIM];    // Measurement noise covariance
float F[STATE_DIM][STATE_DIM];  // State transition matrix
float H[MEAS_DIM][STATE_DIM];   // Measurement matrix