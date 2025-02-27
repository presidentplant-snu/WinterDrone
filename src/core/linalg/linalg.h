
// Maximum N for inverse
#define MAX_N 10

void matrix_add(const float *A, const float *B, float *C, const int m, const int n);
void matrix_multiply(const float *A, const float *B, float *C, const int m, const int n, const int p);
void matrix_transpose(const float *A, float *AT, const int m, const int n);
int matrix_inverse(const float *A, float *Ainv, const int n);
int matrix_inverse_3x3(const float A[3][3], float Ainv[3][3]);
