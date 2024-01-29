#ifndef SRC_H
#define SRC_H

#define SIGN(x) ((x) > 0) - ((x) < 0)
#define CLAMP(value, low, high) ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

void matrix_multi(double *C, double *A, double *B, int rows1, int cols1, int cols2);
void matrix_transpose(int rows, int cols, double matrix[rows][cols], double result[cols][rows]);
void calculate_norm(int rows, int cols, double matrix[rows][cols], double* norm);
void inv_matrix(int n, double matrix[][n], double inv_matrix[][n]);

#endif // SRC_H
