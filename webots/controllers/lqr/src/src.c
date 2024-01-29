#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "src.h"

void matrix_multi(double *C, double *A, double *B, int rows1, int cols1, int cols2){
    for (int j = 0; j < rows1; j++){
        for (int k = 0; k < cols2; k++){
            *(C + j * cols2 + k) = 0.0;
            for (int g = 0; g < cols1; g++){
                *(C + j * cols2 + k) += *(A + j * cols1 + g) * *(B + g * cols2 + k);
            }
        }
    }
}

void matrix_transpose(int rows, int cols, double matrix[rows][cols], double result[cols][rows]){
    for (int j = 0; j < rows; j++){
        for (int k = 0; k < cols; k++){
            result[k][j] = matrix[j][k];
        }
    }
}

void calculate_norm(int rows, int cols, double matrix[rows][cols], double* norm) {
    double norm1 = 0;

    for (int j = 0; j < rows; j++){
        for (int k = 0; k < cols; k++){
            norm1 += pow(matrix[j][k], 2);
        }
    }

    *norm = sqrt(norm1);
}

void inv_matrix(int n, double matrix[][n], double inv_matrix[][n]) {
    double identity[n][n];
    double matrix_p[n][n];
    double ratio, temp;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (i == j)
                identity[i][j] = 1;
            else
                identity[i][j] = 0;

            matrix_p[i][j] = matrix[i][j];
        }
    }

    for (int i = 0; i < n; i++) {
        temp = matrix_p[i][i];
        for (int j = 0; j < n; j++) {
            matrix_p[i][j] /= temp;
            identity[i][j] /= temp;
        }

        for (int j = 0; j < n; j++) {
            if (j != i) {
                ratio = matrix_p[j][i];
                for (int k = 0; k < n; k++) {
                    matrix_p[j][k] -= ratio * matrix_p[i][k];
                    identity[j][k] -= ratio * identity[i][k];
                }
            }
        }
    }

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            inv_matrix[i][j] = identity[i][j];
        }
    }
}
