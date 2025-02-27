#include "linalg.h"
#include <math.h>


/** 
 * Adds two matrices: C = A + B
 * 
 * @param A Input matrix of size m x n
 * @param B Input matrix of size m x n
 * @param C Output matrix of size m x n
 * @param m Number of rows in A,B
 * @param n Number of columns in A,B
 */
void matrix_add(const float *A, const float *B, float *C,
		const int m, const int n) {
    // Initialize C to zeros
    for (int i = 0; i < m * n; i++) {
        C[i] = 0.0f;
    }
    
    // Perform matrix multiplication
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
                C[i * m + j] += A[i * m + j] * B[i * m + j];
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
void matrix_multiply(const float *A, const float *B, float *C,
		const int m, const int n, const int p) {
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
void matrix_transpose(const float *A, float *AT, const int m, const int n) {
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

float augmented[2*MAX_N*MAX_N]={0} ;
int matrix_inverse(const float *A, float *Ainv, const int n) {
    // Create augmented matrix [A|I]
    if (n > MAX_N) {
        // Handle memory allocation failure
        return -1;
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
            // Fill result with identity as a fallback
            for (int row = 0; row < n; row++) {
                for (int col = 0; col < n; col++) {
                    Ainv[row * n + col] = (row == col) ? 1.0f : 0.0f;
                }
            }
            return -1;
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
	return 0;
}


/**
 * Computes the inverse of a 3x3square matrix: Ainv = A^(-1)
 * This implementation uses Cramer's Rule 
 * 
 * @param A Input square matrix of size n x n
 * @param Ainv Output matrix of size n x n
 */
int matrix_inverse_3x3(const float A[3][3], float Ainv[3][3]) {
	for(int i=0; i<3; i++){
	for(int j=0; j<3; j++) Ainv[i][j]=0;
	}

    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
		A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
		A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
	if(fabsf(det) < 1e-5) return -1;
    float inv_det = 1.0f / det;
	
    Ainv[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * inv_det;
    Ainv[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * inv_det;
    Ainv[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * inv_det;
    Ainv[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * inv_det;
    Ainv[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * inv_det;
    Ainv[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * inv_det;
    Ainv[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * inv_det;
    Ainv[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * inv_det;
    Ainv[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * inv_det;
}

