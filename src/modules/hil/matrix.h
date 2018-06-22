/*
 * matrix.h
 *
 *  Created on: Aug 25, 2016
 *      Author: lidq
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>

typedef double num;

typedef struct
{
	int m;
	int n;
	num *v;
} s_Matrix;

int matrix_init(s_Matrix *matrix, int m, int n);

int matrix_destory(s_Matrix *matrix);

int matrix_zero(s_Matrix *matrix);

int matrix_add(s_Matrix *result, s_Matrix *src, s_Matrix *src1);

int matrix_add_num(s_Matrix *result, s_Matrix *src, num v);

int matrix_add_num_slef(s_Matrix *self, num v);

int matrix_mult(s_Matrix *result, s_Matrix *src, s_Matrix *src1);

int matrix_transposition(s_Matrix *result, s_Matrix *src);

int matrix_mult_num(s_Matrix *result, s_Matrix *src, num v);

int matrix_mult_num_self(s_Matrix *src, num v);

int matrix_determinant(num *result, s_Matrix *src);

int matrix_adjoint(s_Matrix *result, s_Matrix *src);

int matrix_inverse(s_Matrix *result, s_Matrix *src);

int matrix_inv(s_Matrix *result, s_Matrix *src);

int matrix_display(s_Matrix *matrix);

#endif /* MATRIX_H_ */
